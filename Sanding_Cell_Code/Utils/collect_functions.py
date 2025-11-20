#Move this file into root directory when executing
import os
import ast
from collections import defaultdict

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))

def iter_python_files(root):
    for dirpath, dirnames, filenames in os.walk(root):
        # Skip venvs / git / etc if you want
        if any(skip in dirpath for skip in {'.git', '.venv', 'venv', '__pycache__'}):
            continue
        for filename in filenames:
            if filename.endswith('.py'):
                yield os.path.join(dirpath, filename)


class FunctionUsageVisitor(ast.NodeVisitor):
    def __init__(self, filename):
        self.filename = filename
        self.defined_functions = set()     # functions defined in this file
        self.called_functions = []         # list of (func_name, lineno)

    def visit_FunctionDef(self, node: ast.FunctionDef):
        self.defined_functions.add(node.name)
        self.generic_visit(node)

    def visit_AsyncFunctionDef(self, node: ast.AsyncFunctionDef):
        self.defined_functions.add(node.name)
        self.generic_visit(node)

    def visit_Call(self, node: ast.Call):
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
    all_defined = defaultdict(set)  # func_name -> set of files defining it
    raw_calls = []                  # list of (func_name, filename, lineno)

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

    # Only keep calls to functions that are defined somewhere in the project
    usage = defaultdict(list)  # func_name -> list of (filename, lineno)
    for func_name, filename, lineno in raw_calls:
        if func_name in all_defined:
            usage[func_name].append((filename, lineno))

    return all_defined, usage


def main():
    all_defined, usage = analyze_project(PROJECT_ROOT)

    print("\n=== Function definitions ===")
    for func_name, files in sorted(all_defined.items()):
        print(f"\n{func_name} defined in:")
        for f in sorted(files):
            print(f"  - {os.path.relpath(f, PROJECT_ROOT)}")

    print("\n\n=== Function usage (calls) ===")
    for func_name, locations in sorted(usage.items()):
        print(f"\n{func_name} is called in:")
        for filename, lineno in sorted(locations):
            rel = os.path.relpath(filename, PROJECT_ROOT)
            print(f"  - {rel}:{lineno}")


if __name__ == "__main__":
    main()