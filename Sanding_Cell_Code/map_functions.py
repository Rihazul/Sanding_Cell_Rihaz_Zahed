import os
import ast
from collections import defaultdict

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))

SKIP_DIRS = {
    '.git', '.venv', 'venv', '__pycache__',
    'Lib', 'lib', 'site-packages'
}


def iter_python_files(root):
    for dirpath, dirnames, filenames in os.walk(root):
        # filter out dirs we don't care about
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]

        for filename in filenames:
            if filename.endswith('.py'):
                yield os.path.join(dirpath, filename)


class FunctionUsageVisitor(ast.NodeVisitor):
    def __init__(self, filename):
        self.filename = filename
        self.defined_functions = set()       # functions defined in this file
        self.called_functions = []           # list of (func_name, lineno)

    def visit_FunctionDef(self, node):
        self.defined_functions.add(node.name)
        self.generic_visit(node)

    def visit_AsyncFunctionDef(self, node):
        self.defined_functions.add(node.name)
        self.generic_visit(node)

    def visit_Call(self, node):
        func_name = None

        # simple call: foo()
        if isinstance(node.func, ast.Name):
            func_name = node.func.id

        # attribute call: obj.foo()
        elif isinstance(node.func, ast.Attribute):
            func_name = node.func.attr

        if func_name:
            self.called_functions.append((func_name, node.lineno))

        self.generic_visit(node)


def analyze_project(root):
    all_defined = defaultdict(set)   # func_name -> {files defining it}
    raw_calls = []                   # list of (func_name, caller_file, lineno)

    for path in iter_python_files(root):
        try:
            with open(path, 'r', encoding='utf-8') as f:
                src = f.read()
            tree = ast.parse(src, filename=path)
        except Exception as e:
            print(f"Could not parse {path}: {e}")
            continue

        visitor = FunctionUsageVisitor(path)
        visitor.visit(tree)

        for fn in visitor.defined_functions:
            all_defined[fn].add(path)

        for fn, lineno in visitor.called_functions:
            raw_calls.append((fn, path, lineno))

    return all_defined, raw_calls


def build_edges(all_defined, raw_calls):
    """
    Build edges between files:
      caller_file -> callee_file  [functions...]
    """
    edges = defaultdict(set)  # (caller_file, callee_file) -> {func_names}

    for func_name, caller_file, lineno in raw_calls:
        if func_name not in all_defined:
            continue  # called function not defined in this project

        for callee_file in all_defined[func_name]:
            # if you don't want same-file edges, uncomment:
            # if caller_file == callee_file:
            #     continue
            edges[(caller_file, callee_file)].add(func_name)

    return edges


def to_rel(path):
    return os.path.relpath(path, PROJECT_ROOT).replace("\\", "/")


def write_txt(all_defined, raw_calls, edges, output_path="function_map.txt"):
    # Build per-function usage map first: func_name -> list of (file, lineno)
    usage = defaultdict(list)
    for func_name, caller_file, lineno in raw_calls:
        if func_name in all_defined:
            usage[func_name].append((caller_file, lineno))

    with open(output_path, "w", encoding="utf-8") as f:
        f.write("=== FUNCTION DEFINITIONS & USAGE ===\n\n")

        for func_name in sorted(all_defined.keys()):
            f.write(f"Function: {func_name}\n")
            f.write("  Defined in:\n")
            for file_path in sorted(all_defined[func_name]):
                f.write(f"    - {to_rel(file_path)}\n")

            if usage.get(func_name):
                f.write("  Used in:\n")
                for caller_file, lineno in sorted(usage[func_name], key=lambda x: (x[0], x[1])):
                    f.write(f"    - {to_rel(caller_file)}:{lineno}\n")
            else:
                f.write("  Used in:\n")
                f.write("    - (no calls found)\n")

            f.write("\n")

        f.write("\n\n=== FILE-TO-FILE CALL GRAPH ===\n\n")
        f.write("Format: source_file -> target_file : func1, func2, ...\n\n")

        # group edges by caller for a cleaner "arrow" style
        by_caller = defaultdict(list)  # caller -> list of (callee, {funcs})
        for (caller, callee), funcs in edges.items():
            by_caller[caller].append((callee, funcs))

        for caller, targets in sorted(by_caller.items(), key=lambda x: to_rel(x[0])):
            caller_label = to_rel(caller)
            f.write(f"[SOURCE] {caller_label}\n")
            for callee, funcs in sorted(targets, key=lambda x: to_rel(x[0])):
                callee_label = to_rel(callee)
                func_list = ", ".join(sorted(funcs))
                f.write(f"  -> {callee_label} : {func_list}\n")
            f.write("\n")

    print(f"Wrote text report: {output_path}")


def main():
    all_defined, raw_calls = analyze_project(PROJECT_ROOT)
    edges = build_edges(all_defined, raw_calls)
    write_txt(all_defined, raw_calls, edges)


if __name__ == "__main__":
    main()