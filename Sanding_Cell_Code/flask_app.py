from flask import Flask, render_template, jsonify, request
from flask_cors import CORS
from threading import Thread
from Server_Better_V2 import handle_client, request_stop, clear_stop
from Server_Better_V2 import getTool11, keepTool11,communicate,laser,stopper_statusmod,set_table_state
from Server_Better_V2 import setup_logger
import yaml, requests
from modules.CPS import CPSClient
from multiprocessing import Process, Event
import time
from flask_socketio import SocketIO
# Get the absolute path to flask_app.py (important when running as an executable)
import os
import sys
import webview  # PyWebView for embedding Flask in a window
from werkzeug.utils import secure_filename
import logging
import threading
from contextlib import contextmanager

# Reduce noisy werkzeug request logs.
logging.getLogger("werkzeug").setLevel(logging.ERROR)
from FileUtils.upload import upload3DModel
from Table1Model.plotexp import plot_data as plot_data_table1Model
from Table2Model.plotexp import plot_data as plot_data_table2Model
from Table3Model.plotexp import plot_data as plot_data_table3Model
from Table4Model.plotexp import plot_data as plot_data_table4Model
from Table5Model.plotexp import plot_data as plot_data_table5Model
import json
from model1cycle.mainmodel1 import startingRobotToSandmodel1
from model2cycle.mainmodel2 import startingRobotToSandmodel2
from model3cycle.mainmodel3 import startingRobotToSandmodel3
from model4cycle.mainmodel4 import startingRobotToSandmodel4
from model5cycle.mainmodel5 import startingRobotToSandmodel5
from smallTable.smallmodelfinal import sandingModelATableA
from smallTable.smallmodelfinal2 import sandingModelBTableA
from smallTable.smallmodelfinal3 import sandingModelCTableA
from smallTable.smallmodelfinal4 import sandingModelDETableA
from smallTable.smallmodelfinal5 import sandingModelETableA
from smallTable.scansmalltable import scanTableA
from smallTable.scanhoming import scanhoming
from smallTable.homingtotal import homingtotal

UPLOAD_FOLDER = './3DModels'
ALLOWED_EXTENSIONS = {'stp'}
os.makedirs(UPLOAD_FOLDER,exist_ok=True)


#RightTable 
modelMethodmap = {
    "modelA": startingRobotToSandmodel1,
    "modelB": startingRobotToSandmodel2,
    "modelC": startingRobotToSandmodel3,
    "modelD": startingRobotToSandmodel4,
    "modelE": startingRobotToSandmodel5,
}
#RightTable
modelMap = {
    "modelA": plot_data_table1Model,
    "modelB": plot_data_table2Model,
    "modelC": plot_data_table3Model,
    "modelD": plot_data_table4Model,
    "modelE": plot_data_table5Model,
}

#left Table
modelMethodMapTableA = {
    "modelA": sandingModelATableA,
    "modelB": sandingModelBTableA,
    "modelC": sandingModelCTableA,
    "modelD": sandingModelDETableA,
    "modelE": sandingModelETableA,
}


def allowed_file(filename):
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS

############################################################################################
client_process = None
app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, async_mode='threading')

######################################################f######################################
default_data = {
    "tableA": {
        "UI": {
            "frameSandCount": 0.0,
            "pocketZigSandCnt": 0.0,
            "pocketSQSandCnt": 0.0,
            "threeDSandCount": 0.0,
            "inedgeSandCount": 0.0,
            "outedgeSandCount": 0.0,
            "sideSandCount": 0.0,
            "robotSpeed": 0.90,
            "scanSpeed": 0.15, # Increased from 0.15 to 0.25 by rafat
            "seventhAxisSpeed": 50,
            "seventhAxisVelocity": 20,
            "model": "1",  # Assuming default model is "1"
            "controlPressure": 0.0,
            "frameForce": 1,
            "pocketZigForce": 1,
            "pocketSQForce": 1,
            "threeDForce": 1,
            "inedgeForce": 1,
            "outedgeForce": 1,
            "sideForce": 1,
        }
    }
}

# Duplicate tableA data in tableB since these are the default data
default_data["tableB"] = default_data["tableA"]

############################################################################################
CPS = CPSClient()
IP = "192.168.0.10"
port = 10003
ret = CPS.HRIF_Connect(0, IP, port)
Tpos = 0
velocity = 0.1
robot_lock = threading.Lock()


def ensure_cps_connected():
    """Ensure the global CPS client is connected before issuing commands."""
    try:
        if CPS.HRIF_IsConnected(0):
            return 0
    except Exception:
        pass
    return CPS.HRIF_Connect(0, IP, port)

@contextmanager
def locked_cps(timeout=0.2, allow_when_busy=False):
    """Lock CPS access and ensure connection; yields True on success."""
    if not allow_when_busy and process_state.get("status") == "in_progress":
        yield False
        return
    acquired = robot_lock.acquire(timeout=timeout)
    if not acquired:
        yield False
        return
    ok = False
    try:
        ret = ensure_cps_connected()
        ok = ret == 0
        yield ok
    finally:
        if acquired:
            robot_lock.release()

############################################################################################
# Store modal data and process state globally
modal_data_store = {}
process_state = {'status': 'completed'}  # Can be 'in_progress' or 'completed'


def _track_process(proc: Process) -> None:
    """Background watcher to clear state when a spawned process finishes."""
    try:
        proc.join()
    finally:
        global client_process
        process_state['status'] = 'completed'
        client_process = None
        socketio.emit('flash_message', {"message": "Process finished"})

############################################################################################
# Home route to render the frontend interface. Add more routes if necessary!
@app.route('/')
def interface():
    return render_template('interface.html')

############################################################################################
# Display the sanding state in the frontend.
@app.route('/trigger', methods=['POST'])
def trigger():
    # Parse the JSON payload
    data = request.json
    if not data:
        return jsonify({"error": "Invalid request, no JSON data found"}), 400

    # Extract the message
    message = data.get('message', 'No message provided')

    # Send the message to all connected WebSocket clients
    socketio.emit('flash_message', {"message": message})

    return jsonify({"status": "Message sent"}), 200

############################################################################################
# Save modal data
@app.route('/save_modal_data', methods=['POST'])
def save_modal_data():
    global modal_data_store    
    try:
        modal_data = request.json  # Get modal data from frontend
        modal_data_store = modal_data  # Store it in memory
        return jsonify({'status': 'success', 'message': 'Modal data saved successfully'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Error saving modal data: {str(e)}'})

############################################################################################
# Get modal data
@app.route('/get_modal_data', methods=['GET'])
def get_modal_data():
    if modal_data_store:
        return jsonify(modal_data_store)
    else:
        return jsonify(default_data)

############################################################################################

@app.route('/start_TableB_process', methods=['POST'])
def start_TableB_process():
    global client_process
    data = request.json

    if client_process and client_process.is_alive():
        return jsonify({'status': 'error', 'message': 'Process already running'})
    
    if not data:
        return jsonify({"error": "Invalid request, no JSON data found"}), 400
    
    if 'TableB' not in data:
        return jsonify({"error": "Invalid request, no JSON data found"}), 400
    
    tableData = data['TableB']
    
    if 'model' not in tableData:
        return jsonify({"error": "Invalid request, no JSON data found"}), 400

    buttonClicked = modelMap[tableData['model']](inverseOverlapping=data.get('inverseOverlapping', 50))
    with open('./configs/cycleData.json', 'w') as f:
        json.dump(data, f)
        
    if buttonClicked == "cancel":
        return jsonify({
            'success': False,
            'process' : "cancelled",
            "status": "Process is cancelled"
        })   
    
    set_table_state(CPS, "tableAOpenClose", "Close")
    set_table_state(CPS, "tableBOpenClose", "Close")
    nRet = CPS.HRIF_SetBoxCO(0, 2, 0)
    print("stopper Test")

    client_process = Process(target=modelMethodmap[tableData['model']], args=())
    process_state['status'] = 'in_progress'
    client_process.start()
    Thread(target=_track_process, args=(client_process,), daemon=True).start()
    
    # startingRobotToSand()
    return jsonify({
        'success': True,
        'process' : "started",
        "status": "Process started"
    })

############################################################################################

@app.route('/start_TableA_process', methods=['POST'])
def start_TableA_process():
    global client_process
    data = request.json
    print("Received data in start_TableA_process:", data)
    
    if client_process and client_process.is_alive():
        return jsonify({'status': 'error', 'message': 'Process already running'})

    if not data:
        return jsonify({"error": "Invalid request, no JSON data found"}), 400
    
    if 'TableA' not in data:
        return jsonify({"error": "Invalid request, no JSON data found"}), 400
    
    tableData = data['TableA']

    # Support both payload formats:
    # 1) Legacy: TableA.model + task objects
    # 2) Door-based: TableA.doors[] where each door can include model + task objects
    selected_model = tableData.get('model')
    if not selected_model or selected_model == 'None':
        doors = tableData.get('doors')
        if isinstance(doors, list) and doors:
            # Infer the model from the first door that has one.
            selected_model = next((d.get('model') for d in doors if isinstance(d, dict) and d.get('model')), None)
            # If the frontend sends model keys like modelA/modelB..., use that.
        if not selected_model or selected_model == 'None':
            return jsonify({"error": "Invalid request, no model found for TableA"}), 400
        # Make it available to any downstream code that still expects TableA.model
        tableData['model'] = selected_model
    
    with open('./configs/cycleData.json', 'w') as f:
        json.dump(data, f)

    set_table_state(CPS, "tableAOpenClose", "Close")
    set_table_state(CPS, "tableBOpenClose", "Close")
    stopper_statusmod(CPS, state="up")

    client_process = Process(target=modelMethodMapTableA[tableData['model']], args=())
    process_state['status'] = 'in_progress'
    client_process.start()
    Thread(target=_track_process, args=(client_process,), daemon=True).start()
    
    return jsonify({
        'success': True,
        'process' : "started",
        "status": "Process started"
    })
    

############################################################################################



@app.route('/upload_3d_file', methods=['POST'])
def upload_3d_file():
    if 'file' not in request.files:
        return jsonify({'success': False, 'message': 'No file part'})
    
    file = request.files['file']
    if file.filename == '':
        return jsonify({'success': False, 'message': 'No selected file'})
        
    
    if file and allowed_file(file.filename):
        try:
            # Secure the filename and save the file
            filename = secure_filename(file.filename)
            filepath = os.path.join(UPLOAD_FOLDER, filename)
            file.save(filepath)
            
            upload3DModel(filepath) 
            
            return jsonify({
                'success': True,
                'message': 'File uploaded successfully',
                'filename': filename                
            })
        except Exception as e:
            return jsonify({
                'success': False,
                'message': f'Error processing file: {str(e)}'
            })
    
    return jsonify({
        'success': False,
        'message': 'Invalid file type'
    })
    
    
############################################################################################

# Combines the data from the config file and the user inputted data and return as combined configurations data.
def fetch_and_combine_data():
    try:
        api_response = requests.get("http://localhost:5100/get_modal_data")
        api_data = api_response.json()
        
        if api_data.get("message") == "No modal data.":
            return {"status": "incomplete", "message": "Settings are still remaining to do."}
    except Exception as e:
        return {"status": "error", "message": f"API error: {e}"}
    
    try:
        with open('./configs/config.yaml', 'r') as file:
            config_data = yaml.safe_load(file)
        print("Config Data: ", config_data)
    except Exception as e:
        return {"status": "error", "message": f"YAML file error: {e}"}
    
    combined_data = {**config_data, **api_data['tableA']}  # YAML data is combined with API data
    return combined_data

############################################################################################
# Start the handle_client process threads
# def start_process(config):
#     global client_process
#     if client_process and client_process.is_alive():
#         return jsonify({'status': 'error', 'message': 'Process already running'})

#     client_process = Process(target=handle_client, args=(config,))
#     client_process.start()
#     socketio.emit('flash_message', {"message": f"Scanning and Sanding Started..."})
#     return jsonify({'status': 'success', 'message': 'Process started successfully'})

############################################################################################
# Stops the handle_client process threads if stop button is pressed!

@app.route("/stop", methods=["POST"])
def stop_process():
    if client_process and client_process.is_alive():
        client_process.terminate()  # Immediately terminate the process
        # client_process.join()  # Wait for the process to finish
        print("Client process terminated.")
        process_state['status'] = 'completed'
        globals()['client_process'] = None
        socketio.emit('flash_message', {"message": f"Stopped All Running Process!"})
        return jsonify({'status': 'success', 'message': 'Process terminated successfully'})
    else:
        print("No active client process to terminate.")
        socketio.emit('flash_message', {"message": f"No Process to Stop"})
        return jsonify({'status': 'error', 'message': 'No active process to terminate'})

############################################################################################
#Tool3_pick_and_drop integration by rafat
@app.route("/tool_toggle", methods=["POST"])
def tool_toggle():
    """
    Expects a JSON payload like:
    {
      "toolNumber": 1,
      "action": "pick"  (or "keep")
    }
    """
    data = request.json
    if not data:
        return jsonify({"error": "No JSON data provided"}), 400

    tool_num = data.get("toolNumber")
    action = data.get("action")

    if not tool_num or not action:
        return jsonify({"error": "toolNumber or action missing"}), 400

    # 1) Get or build your config from file + modal data
    config_data_UI = fetch_and_combine_data()
    
    # Manually add a logger instance to config
    config_data_UI['logger'] = setup_logger(config_data_UI['settings']['debug'])
    
    # 2) Use the shared CPS client
    cps = CPS

    def check_conditions(condition_list):
        """Helper to check CPS conditions."""
        for func, box_id, nBit, expected in condition_list:
            print(func, box_id, nBit, expected)
            result = []
            nRet = func(box_id, nBit, result)
            print('output', nRet, result)
            if nRet != 0 or result[0] != str(expected):
                return False
        return True

    # 3) Depending on the action, call getTool or keepTool
    if action == "pick":
        with locked_cps() as ok:
            if not ok:
                return jsonify({"error": "Failed to connect to CPS client"}), 500
            # Validate Pick Conditions
            pick_conditions = [
                (cps.HRIF_ReadBoxCI, 0, 0, 0),
                (cps.HRIF_ReadBoxCI, 0, 1, 0),
                (cps.HRIF_ReadBoxCI, 0, 2, 0),
                (cps.HRIF_ReadBoxDI, 0, 4, 1)
            ]
            if check_conditions(pick_conditions):
                # Pick the tool
                getTool11(cps, toolNumber=tool_num, config=config_data_UI)
                socketio.emit('flash_message', {"message": f"Picked Tool {tool_num}"})
                # cps.HRIF_DisConnect(0)
                return jsonify({"status": "success", "message": f"Tool {tool_num} picked successfully"})
            else:
                socketio.emit('flash_message', {"message": f"Already holding a tool. Can't pick Tool {tool_num}"})
                # cps.HRIF_DisConnect(0)
                return jsonify({"error": "Pick conditions not met"}), 400

    elif action == "keep":
        with locked_cps() as ok:
            if not ok:
                return jsonify({"error": "Failed to connect to CPS client"}), 500
            # Validate Drop Conditions
            drop_conditions = [
                (cps.HRIF_ReadBoxCI, 0, 0, 1),
                (cps.HRIF_ReadBoxCI, 0, 1, 0),
                (cps.HRIF_ReadBoxCI, 0, 2, 0),
                (cps.HRIF_ReadBoxDI, 0, 4, 0)
            ]
            if check_conditions(drop_conditions):
                # Keep the tool
                keepTool11(cps, toolNumber=tool_num, config=config_data_UI)
                socketio.emit('flash_message', {"message": f"Kept Tool {tool_num}"})
                # cps.HRIF_DisConnect(0)
                return jsonify({"status": "success", "message": f"Tool {tool_num} kept successfully"})
            else:
                socketio.emit('flash_message', {"message": f"Not holding Tool {tool_num} to drop"})
                # cps.HRIF_DisConnect(0)
                return jsonify({"error": "Drop conditions not met"}), 400

    else:
        return jsonify({"error": "Invalid action. Must be 'pick' or 'keep'."}), 400


############################################################################################
#Tool2_pick_and_drop integration by rafat
@app.route("/tool_toggle2", methods=["POST"])
def tool_toggle2():
    """
    Expects a JSON payload like:
    {
      "toolNumber": 2,
      "action": "pick"  (or "keep")
    }
    """
    data = request.json
    if not data:
        return jsonify({"error": "No JSON data provided"}), 400

    tool_num = data.get("toolNumber")
    action = data.get("action")
    print(f'tool num: {tool_num}, action: {action}')
    if not tool_num or not action:
        return jsonify({"error": "toolNumber or action missing"}), 400

    # 1) Get or build your config from file + modal data
    config_data_UI = fetch_and_combine_data()
    
    # Manually add a logger instance to config
    config_data_UI['logger'] = setup_logger(config_data_UI['settings']['debug'])
    
    # 2) Use the shared CPS client
    cps = CPS

    def check_conditions(condition_list):
        """Helper to check CPS conditions."""
        for func, box_id, nBit, expected in condition_list:
            print(func, box_id, nBit, expected)
            result = []
            nRet = func(box_id, nBit, result)
            print('output t2', nRet, result)
            if nRet != 0 or result[0] != str(expected):
                return False
        return True

    # 3) Depending on the action, call getTool or keepTool
    if action == "pick":
        with locked_cps() as ok:
            if not ok:
                return jsonify({"error": "Failed to connect to CPS client"}), 500
            # Validate Pick Conditions
            pick_conditions = [
                (cps.HRIF_ReadBoxCI, 0, 0, 0),
                (cps.HRIF_ReadBoxCI, 0, 1, 0),
                (cps.HRIF_ReadBoxCI, 0, 2, 0),
                (cps.HRIF_ReadBoxDI, 0, 5, 1)
            ]
            if check_conditions(pick_conditions):
                # Pick the tool
                getTool11(cps, toolNumber=tool_num, config=config_data_UI)
                socketio.emit('flash_message', {"message": f"Picked Tool {tool_num}"})
                # cps.HRIF_DisConnect(0)
                return jsonify({"status": "success", "message": f"Tool {tool_num} picked successfully"})
            else:
                socketio.emit('flash_message', {"message": f"Already holding a tool. Can't pick Tool {tool_num}"})
                # cps.HRIF_DisConnect(0)
                return jsonify({"error": "Pick conditions not met"}), 400

    elif action == "keep":
        with locked_cps() as ok:
            if not ok:
                return jsonify({"error": "Failed to connect to CPS client"}), 500
            # Validate Drop Conditions
            drop_conditions = [
                (cps.HRIF_ReadBoxCI, 0, 0, 0),
                (cps.HRIF_ReadBoxCI, 0, 1, 1),
                (cps.HRIF_ReadBoxCI, 0, 2, 0),
                (cps.HRIF_ReadBoxDI, 0, 5, 0)
            ]
            if check_conditions(drop_conditions):
                # Keep the tool
                keepTool11(cps, toolNumber=tool_num, config=config_data_UI)
                socketio.emit('flash_message', {"message": f"Kept Tool {tool_num}"})
                # cps.HRIF_DisConnect(0)
                return jsonify({"status": "success", "message": f"Tool {tool_num} kept successfully"})
            else:
                socketio.emit('flash_message', {"message": f"Not holding Tool {tool_num} to drop"})
                # cps.HRIF_DisConnect(0)
                return jsonify({"error": "Drop conditions not met"}), 400
    else:
        return jsonify({"error": "Invalid action. Must be 'pick' or 'keep'."}), 400
    

###########################################################################################
############################################################################################
#Tool1_pick_and_drop integration by rafat
@app.route("/tool_toggle1", methods=["POST"])
def tool_toggle1():
    """
    Expects a JSON payload like:
    {
      "toolNumber": 1,
      "action": "pick"  (or "keep")
    }
    """
    data = request.json
    if not data:
        return jsonify({"error": "No JSON data provided"}), 400

    tool_num = data.get("toolNumber")
    action = data.get("action")

    if not tool_num or not action:
        return jsonify({"error": "toolNumber or action missing"}), 400

    # 1) Get or build your config from file + modal data
    config_data_UI = fetch_and_combine_data()
    
    # Manually add a logger instance to config
    config_data_UI['logger'] = setup_logger(config_data_UI['settings']['debug'])
    
    # 2) Use the shared CPS client
    cps = CPS

    def check_conditions(condition_list):
        """Helper to check CPS conditions."""
        print(condition_list)
        for func, box_id, nBit, expected in condition_list:
            result = []
            nRet = func(box_id, nBit, result)
            print('output tool_toggle1', nRet, result)
            if nRet != 0 or result[0] != str(expected):
                return False
        return True

    # 3) Depending on the action, call getTool or keepTool
    if action == "pick":
        with locked_cps() as ok:
            if not ok:
                return jsonify({"error": "Failed to connect to CPS client"}), 500
            # Validate Pick Conditions
            pick_conditions = [
                (cps.HRIF_ReadBoxCI, 0, 0, 0),
                (cps.HRIF_ReadBoxCI, 0, 1, 0),
                (cps.HRIF_ReadBoxCI, 0, 2, 0),
                (cps.HRIF_ReadBoxDI, 0, 7, 1),
            ]
            if check_conditions(pick_conditions):
                # Pick the tool
                getTool11(cps, toolNumber=tool_num, config=config_data_UI)
                socketio.emit('flash_message', {"message": f"Picked Tool {tool_num}"})
                # cps.HRIF_DisConnect(0)
                return jsonify({"status": "success", "message": f"Tool {tool_num} picked successfully"})
            else:
                socketio.emit('flash_message', {"message": f"Already holding a tool. Can't pick Tool {tool_num}"})
                # cps.HRIF_DisConnect(0)
                return jsonify({"error": "Pick conditions not met"}), 400

    elif action == "keep":
        with locked_cps() as ok:
            if not ok:
                return jsonify({"error": "Failed to connect to CPS client"}), 500
            # Validate Drop Conditions
            drop_conditions = [
                (cps.HRIF_ReadBoxCI, 0, 0, 0),
                (cps.HRIF_ReadBoxCI, 0, 1, 1),
                (cps.HRIF_ReadBoxCI, 0, 2, 1),
                (cps.HRIF_ReadBoxDI, 0, 7, 0)
            ]
            if check_conditions(drop_conditions):
                # Keep the tool
                keepTool11(cps, toolNumber=tool_num, config=config_data_UI)
                socketio.emit('flash_message', {"message": f"Kept Tool {tool_num}"})
                # cps.HRIF_DisConnect(0)
                return jsonify({"status": "success", "message": f"Tool {tool_num} kept successfully"})
            else:
                socketio.emit('flash_message', {"message": f"Not holding Tool {tool_num} to drop"})
                # cps.HRIF_DisConnect(0)
                return jsonify({"error": "Drop conditions not met"}), 400
    else:
        return jsonify({"error": "Invalid action. Must be 'pick' or 'keep'."}), 400

############################################################################################
def check_tool1_attachment_condition(cps):
    """
    Checks if these conditions are met:
        (cps.HRIF_ReadBoxCI, 0, 0, 0)
        (cps.HRIF_ReadBoxCI, 0, 1, 1)
        (cps.HRIF_ReadBoxCI, 0, 2, 1)
        (cps.HRIF_ReadBoxDI, 0, 7, 0)
    If true, signals to the frontend that the circle button should blink
    (or turn green+blink). If false, it tells the frontend to stop blinking.
    """
    condition_list = [
        (cps.HRIF_ReadBoxCI, 0, 0, 0),
        (cps.HRIF_ReadBoxCI, 0, 1, 1),
        (cps.HRIF_ReadBoxCI, 0, 2, 1),
        (cps.HRIF_ReadBoxDI, 0, 7, 0),
    ]

    all_met = True
    for func, box_id, bit, expected_value in condition_list:
        result = []
        ret = func(box_id, bit, result)
        if ret != 0 or result[0] != str(expected_value):
            all_met = False
            break

    if all_met:
        socketio.emit('blink_circle_button', {'shouldBlink': True})
    else:
        socketio.emit('blink_circle_button', {'shouldBlink': False})

    return all_met
############################################################################################
@app.route('/check_tool1_status', methods=['GET'])
def check_tool1_status():
    """
    A route that calls our check_tool1_attachment_condition function
    and returns a simple JSON response. 
    By calling this route, we ensure the server checks the condition 
    and emits the blink_circle_button event if needed.
    """
    with locked_cps() as ok:
        if not ok:
            return jsonify({"status": "busy", "shouldBlink": False})
        should_blink = check_tool1_attachment_condition(CPS)  # Emits if any WS clients exist.
    return jsonify({"status": "OK", "shouldBlink": bool(should_blink)})

############################################################################################
# For Tool 2 Status function
def check_tool2_attachment_condition(cps):
    """
    Checks if these conditions are met for Tool 2 attachment:
        (cps.HRIF_ReadBoxCI, 0, 0, 0)
        (cps.HRIF_ReadBoxCI, 0, 1, 1)
        (cps.HRIF_ReadBoxCI, 0, 2, 0)
        (cps.HRIF_ReadBoxDI, 0, 5, 0)

    If all are true, we emit 'blink_circle_button2' with shouldBlink=True.
    Otherwise, shouldBlink=False.
    """
    condition_list = [
        (cps.HRIF_ReadBoxCI, 0, 0, 0),
        (cps.HRIF_ReadBoxCI, 0, 1, 1),
        (cps.HRIF_ReadBoxCI, 0, 2, 0),
        (cps.HRIF_ReadBoxDI, 0, 5, 0),
    ]

    all_met = True
    for func, box_id, bit, expected_value in condition_list:
        result = []
        ret = func(box_id, bit, result)
        if ret != 0 or result[0] != str(expected_value):
            all_met = False
            break

    if all_met:
        socketio.emit('blink_circle_button2', {'shouldBlink': True})
    else:
        socketio.emit('blink_circle_button2', {'shouldBlink': False})

    return all_met
############################################################################################
@app.route('/check_tool2_status', methods=['GET'])
def check_tool2_status():
    """
    A route that calls check_tool2_attachment_condition.
    By calling this route, we check the lines for tool2 
    and emit a 'blink_circle_button2' event with True or False.
    """
    with locked_cps() as ok:
        if not ok:
            return jsonify({"status": "busy", "shouldBlink": False})
        should_blink = check_tool2_attachment_condition(CPS)
    return jsonify({"status": "OK", "shouldBlink": bool(should_blink)})

############################################################################################
def check_tool3_attachment_condition(cps):
    """
    Checks if these conditions are met for Tool 3 attachment:
        (cps.HRIF_ReadBoxCI, 0, 0, 1)
        (cps.HRIF_ReadBoxCI, 0, 1, 0)
        (cps.HRIF_ReadBoxCI, 0, 2, 0)
        (cps.HRIF_ReadBoxDI, 0, 4, 0)

    If all are true, we emit 'blink_circle_button3' with shouldBlink=True.
    Otherwise, shouldBlink=False.
    """
    condition_list = [
        (cps.HRIF_ReadBoxCI, 0, 0, 1),
        (cps.HRIF_ReadBoxCI, 0, 1, 0),
        (cps.HRIF_ReadBoxCI, 0, 2, 0),
        (cps.HRIF_ReadBoxDI, 0, 4, 0),
    ]

    all_met = True
    for func, box_id, bit, expected_value in condition_list:
        result = []
        ret = func(box_id, bit, result)
        if ret != 0 or result[0] != str(expected_value):
            all_met = False
            break

    if all_met:
        socketio.emit('blink_circle_button3', {'shouldBlink': True})
    else:
        socketio.emit('blink_circle_button3', {'shouldBlink': False})

    return all_met
############################################################################################
@app.route('/check_tool3_status', methods=['GET'])
def check_tool3_status():
    """
    A route that calls check_tool3_attachment_condition.
    By calling this route, we check the lines for tool3 
    and emit a 'blink_circle_button3' event with True or False.
    """
    with locked_cps() as ok:
        if not ok:
            return jsonify({"status": "busy", "shouldBlink": False})
        should_blink = check_tool3_attachment_condition(CPS)
    return jsonify({"status": "OK", "shouldBlink": bool(should_blink)})
############################################################################################
# Toggle the Table A/B Open/Close button text depending upon table current state.Improved by rafat and working
@app.route('/toggle_state/<table_id>', methods=['GET'])
def toggle_state(table_id):
    robot_state = []
    di_state_0 = []
    di_state_1 = []
    with locked_cps() as ok:
        if not ok:
            return jsonify({'newState': 'Busy'}), 200
        if table_id == "tableAOpenClose": 
            #nRet = CPS.HRIF_ReadBoxDO(0, 1, robot_state)
            nRet = CPS.HRIF_ReadBoxDI(0, 0, di_state_0)
            nRet = CPS.HRIF_ReadBoxDI(0, 1, di_state_1)
            if di_state_0[0] == '1' and di_state_1[0] == '0':
                new_state = "Open"
                nRet = CPS.HRIF_SetBoxDO(0, 1, 0) # (0, digital output number, states)
                nRet = CPS.HRIF_SetBoxDO(0, 0, 1)
                # socketio.emit('flash_message', {"message": f"Table A is in horizontal position Status confirmed by sensor"})
            elif di_state_0[0] == '0' and di_state_1[0] == '1':
                new_state = "Close"
                nRet = CPS.HRIF_SetBoxDO(0, 0, 0)
                nRet = CPS.HRIF_SetBoxDO(0, 1, 1)
                socketio.emit('flash_message', {"message": f"Alert !!! Table A is in 45 degree working position so be carefull at the time of manually moving robot"})
            else:
                socketio.emit('flash_message', {"message": f"Warning !! Table is not working .Check your proximity Sensor and physical Wire Connectivity", "type":"warning"})

            #if robot_state[0] == '1':
            
                #new_state = "Open"
                #nRet = CPS.HRIF_SetBoxDO(0, 1, 0) # (0, digital output number, states)
                #nRet = CPS.HRIF_SetBoxDO(0, 0, 1)
            #else:
                #new_state = "Close"
                #nRet = CPS.HRIF_SetBoxDO(0, 0, 0)
                #nRet = CPS.HRIF_SetBoxDO(0, 1, 1)

        elif table_id == "tableBOpenClose":
            nRet = CPS.HRIF_ReadBoxCO(0, 1, robot_state)
            if robot_state[0] == '1':
                new_state = "Open"
                nRet = CPS.HRIF_SetBoxCO(0, 1, 0)  # (0, digital output number, states)
                nRet = CPS.HRIF_SetBoxCO(0, 0, 1)
                socketio.emit('flash_message', {"message": f"Table B is in horizontal position"})
            else:
                new_state = "Close"
                nRet = CPS.HRIF_SetBoxCO(0, 0, 0)
                nRet = CPS.HRIF_SetBoxCO(0, 1, 1)
                socketio.emit('flash_message', {"message": f"Alert !!! Table B is in 45 degree working position so be carefull at the time of manually moving robot"})
            ######
            #ToDo: The project was not complete, so change the digital number in the CPS functions which can open or close the tableB.
            #Syntax: HRIF_SetBoxDo(BoxID, DigitalNumber(Ask Nic What is the Digital Output for TableB), TableState(1or0)) 
            ######
            # nRet = CPS.HRIF_ReadBoxDO(0, 1, robot_state)
            # if robot_state[0] == '1':
            #     new_state = "Open"
            #     nRet = CPS.HRIF_SetBoxDO(0, 1, 0)
            # else:
            #     new_state = "Close"
            #     nRet = CPS.HRIF_SetBoxDO(0, 1, 1)

        else:
            return jsonify({'newState': 'Invalid input. No state change.'})

    return jsonify({'newState': f"{new_state}"})

############################################################################################
# Send the Table A/B actual state to the frontend.

@app.route('/get_state/<table_id>', methods=['GET'])
def get_state(table_id):
    robot_state = []
    with locked_cps() as ok:
        if not ok:
            return jsonify({'newState': 'Busy'}), 200

        if table_id == "tableAOpenClose":
            nRet = CPS.HRIF_ReadBoxDO(0, 1, robot_state)
            socketio.emit('flash_message', {"message": f"Table A is Set To {'Open' if robot_state[0] == '1' else 'Close'}. Please Wait Till The Process Finishes..."})
            if robot_state[0] == '1':
                new_state = "Close"
            else:
                new_state = "Open"

        elif table_id == "tableBOpenClose": 
            nRet = CPS.HRIF_ReadBoxCO(0, 1, robot_state)
            socketio.emit('flash_message', {"message": f"Table B is Set To {'Open' if robot_state[0] == '1' else 'Close'}. Please Wait Till The Process Finishes..."})
            if robot_state[0] == '1':
                new_state = "Close"
            else:
                new_state = "Open"
        else:
            return jsonify({'newState': 'Invalid input. No state change.'})

    
    return jsonify({'newState': f"{new_state}"})

############################################################################################
# Send the Table A/B actual state to the frontend without emitting flash messages.
@app.route('/table_state/<table_id>', methods=['GET'])
def table_state(table_id):
    with locked_cps() as ok:
        if not ok:
            return jsonify({'state': 'Unknown'})
        if table_id == "tableAOpenClose":
            di_state_0 = []
            di_state_1 = []
            nRet0 = CPS.HRIF_ReadBoxDI(0, 0, di_state_0)
            nRet1 = CPS.HRIF_ReadBoxDI(0, 1, di_state_1)
            if nRet0 == 0 and nRet1 == 0 and di_state_0 and di_state_1:
                if di_state_0[0] == '1' and di_state_1[0] == '0':
                    return jsonify({'state': 'Open'})
                if di_state_0[0] == '0' and di_state_1[0] == '1':
                    return jsonify({'state': 'Close'})
            return jsonify({'state': 'Unknown'})

        if table_id == "tableBOpenClose":
            robot_state = []
            nRet = CPS.HRIF_ReadBoxCO(0, 1, robot_state)
            if nRet == 0 and robot_state:
                return jsonify({'state': 'Open' if robot_state[0] == '1' else 'Close'})
            return jsonify({'state': 'Unknown'})

        return jsonify({'state': 'Invalid'})

############################################################################################
# Send the Stopper A/B actual state to the frontend.
@app.route('/stopper_state/<stopper_id>', methods=['GET'])
def stopper_state(stopper_id):
    with locked_cps() as ok:
        if not ok:
            return jsonify({'state': 'Unknown'})
        if stopper_id == "A":
            robot_state = []
            nRet = CPS.HRIF_ReadBoxDO(0, 2, robot_state)
            if nRet == 0 and robot_state:
                return jsonify({'state': 'Up' if robot_state[0] == '1' else 'Down'})
            return jsonify({'state': 'Unknown'})

        if stopper_id == "B":
            robot_state = []
            nRet = CPS.HRIF_ReadBoxCO(0, 2, robot_state)
            if nRet == 0 and robot_state:
                return jsonify({'state': 'Up' if robot_state[0] == '1' else 'Down'})
            return jsonify({'state': 'Unknown'})

        return jsonify({'state': 'Invalid'})

############################################################################################
# Send robot enable/power status to the frontend.
@app.route('/robot_status', methods=['GET'])
def robot_status():
    result = []
    with locked_cps() as ok:
        if not ok:
            return jsonify({'status': 'busy', 'flags': {}})
        nRet = CPS.HRIF_ReadRobotState(0, 0, result)
    if nRet != 0 or len(result) < 13:
        return jsonify({'status': 'error', 'message': 'Failed to read robot state'}), 500

    flags = {
        'in_motion': result[0] == "1",
        'enabled': result[1] == "1",
        'has_error': result[2] == "1",
        'error_code': result[3],
        'error_axis': result[4],
        'lock_state': result[5] == "1",
        'paused': result[6] == "1",
        'emergency_stop': result[7] == "1",
        'light_screen': result[8] == "1",
        'powered_on': result[9] == "1",
        'ebox_connected': result[10] == "1",
        'point_motion_done': result[11] == "1",
        'in_position': result[12] == "1",
    }
    return jsonify({'status': 'ok', 'flags': flags})

def load_config():
    """Loads configuration from config.yaml."""
    with open('./configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    return config

############################################################################################
# Receives all the button actions and send to the respective functions or trigger the functions in the SDK.
@app.route('/action', methods=['POST'])
def handle_action():

    # Load configuration from YAML
    config = load_config()

    # Set up logger
    config['logger'] = setup_logger(config['settings']['debug'])

    action = request.json.get('action')  # Get action from frontend
    print('the action is :',action)
    result = []  # Define the return value as an empty list
    nRet = CPS.HRIF_ReadRobotState(0, 0, result)

    if action == "stopperUp":
        nRet = CPS.HRIF_SetBoxDO(0, 2, 1) #Table Digital Output is 2
        socketio.emit('flash_message', {"message": f"StopperA Put Up"})
        return jsonify({'status': 'success', 'message': 'Action stopper received and executed'})
    
    if action == "stopperDown":
        nRet = CPS.HRIF_SetBoxDO(0, 2, 0) #Table Digital Output is 2
        socketio.emit('flash_message', {"message": f"StopperA Pulled Down"})
        return jsonify({'status': 'success', 'message': 'Action stopper received and executed'})
    
    elif action == "stopperUpB":
        nRet = CPS.HRIF_SetBoxCO(0, 2, 1) #Table Digital Output is 2
        socketio.emit('flash_message', {"message": f"StopperB Put Up"})
        return jsonify({'status': 'success', 'message': 'Action stopper received and executed'})
    
    elif action == "stopperDownB":
        nRet = CPS.HRIF_SetBoxCO(0, 2, 0) #Table Digital Output is 2
        socketio.emit('flash_message', {"message": f"StopperBS Pulled Down"})
        return jsonify({'status': 'success', 'message': 'Action stopper received and executed'})

    elif action == "toolLift":
        nRet = CPS.HRIF_SetBoxDO(0, 5, 0)
        socketio.emit('flash_message', {"message": f"Grabbing the Tool In Hand"})
        return jsonify({'status': 'success', 'message': 'Action tool received and executed'})
    
    elif action == "toolDrop":
        nRet = CPS.HRIF_SetBoxDO(0, 5, 1)
        socketio.emit('flash_message', {"message": f"Dropping the Tool In Hand"})
        return jsonify({'status': 'success', 'message': 'Action tool received and executed'})

    elif action == "stop":
        # Safe stop: stop motion only; do not toggle tool/valve outputs.
        request_stop()
        CPS.HRIF_GrpStop(0, 0)
        try:
            CPS.HRIF_HRApp(0, "HR_Motor", "MotorStop", ["J7"], result)
        except Exception:
            pass
        # Turn off vibration (DO4) on stop.
        CPS.HRIF_SetBoxDO(0, 4, 0)
        laser(CPS, "off", config=config)
        return stop_process()

    elif action == "homing":
        clear_stop()
        # Call your homing function here
        config_data_UI = fetch_and_combine_data()
        socketio.emit('flash_message', {"message": f"Homing Process Started"})
        handle_client(config_data_UI, homingState=True, startSanding=False)
        homingtotal()
        return jsonify({'status': 'success', 'message': 'Action homing received and executed'})

    elif action == "enable":
        nRet = CPS.HRIF_GrpReset(0, 0)
        nRet = CPS.HRIF_GrpEnable(0, 0)
        socketio.emit('flash_message', {"message": f"Cobot Enabled"})
        return jsonify({'status': 'success', 'message': 'Action enable received and executed'})
    
    elif action == "disable":
        nRet = CPS.HRIF_GrpDisable(0, 0)
        socketio.emit('flash_message', {"message": f"Cobot Disabled"})
        return jsonify({'status': 'success', 'message': 'Action enable received and executed'})
    
    # elif action == "confirm":
    #     config_data_UI = fetch_and_combine_data()
    #     return start_process(config_data_UI)
    
    elif action == "scan":
        clear_stop()
        config_data_UI = fetch_and_combine_data()
        with robot_lock:
            ret = ensure_cps_connected()
            if ret != 0:
                print(f"Failed to connect, error code: {ret}")
                exit()
            cps = CPS

        def read_ci_bit(cps, bit_index):
            """Reads CI bit and returns its value (0 or 1)."""
            result = []
            nRet = cps.HRIF_ReadBoxCI(0, bit_index, result)
            if nRet != 0:
                print(f"Error reading CI[{bit_index}], error code: {nRet}")
                return None
            return int(result[0])

        with robot_lock:
            # Read CI values
            ci0 = read_ci_bit(cps, 0)
            ci1 = read_ci_bit(cps, 1)
            ci2 = read_ci_bit(cps, 2)

            # Check Conditions
            if ci0 is None or ci1 is None or ci2 is None:
                print("Failed to read one or more CI bits.")
            else:
                if ci0 == 0 and ci1 == 0 and ci2 == 0:
                    print("No tool in hand")
                    stopper_statusmod(cps, state="up")
                    laser(cps, "on",config=config)
                    set_table_state(CPS, "tableAOpenClose", "Close")
                    set_table_state(CPS, "tableBOpenClose", "Close")
                    scanTableA(cps=CPS, config=config)
                    laser(cps, "off",config=config)
                

                elif ci0 == 1 and ci1 == 0 and ci2 == 0:
                    print("Tool 3 detected → executing keepTool11()")
                    keepTool11(cps, toolNumber=3, config=config)
                    communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=0.9, wait=True)
                    stopper_statusmod(cps, state="up")
                    laser(cps, "on",config=config)
                    set_table_state(CPS, "tableAOpenClose", "Close")
                    set_table_state(CPS, "tableBOpenClose", "Close")
                    scanTableA(cps=CPS, config=config)
                    laser(cps, "off",config=config)

                elif ci0 == 0 and ci1 == 1 and ci2 == 0:
                    print("Tool 2 detected → executing keepTool11()")
                    keepTool11(cps, toolNumber=2, config=config)
                    communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=0.9, wait=True)
                    stopper_statusmod(cps, state="up")
                    laser(cps, "on",config=config)
                    set_table_state(CPS, "tableAOpenClose", "Close")
                    set_table_state(CPS, "tableBOpenClose", "Close")
                    scanTableA(cps=CPS, config=config)
                    laser(cps, "off",config=config)

                elif ci0 == 0 and ci1 == 1 and ci2 == 1:
                    print("Tool 1 detected → executing keepTool11()")
                    keepTool11(cps, toolNumber=1, config=config)
                    communicate(cps=cps, point=config['point']['safePoint'], tcp=config['coords']['tcpDefault'], ucs=config['coords']['ucsDefault'], seventh=-1, config=config, speed=0.9, wait=True)
                    stopper_statusmod(cps, state="up")
                    laser(cps, "on",config=config)
                    set_table_state(CPS, "tableAOpenClose", "Close")
                    set_table_state(CPS, "tableBOpenClose", "Close")
                    scanTableA(cps=CPS, config=config)
                    laser(cps, "off",config=config)

                else:
                    print(f"Unrecognized CI combination: CI0={ci0}, CI1={ci1}, CI2={ci2}")
                # scanTableA() #Run the Table Scan
                # scanhoming() #Run the homing after table scan
                # # handle_client(config_data_UI, homingState=False, startSanding=False, scan=True)
                # return jsonify({'status': 'success', 'message':'Table scan started'})
            scanhoming()
            # scanTableA() #Run the Table Scan
            # scanhoming() #Run the homing after table scan
            # handle_client(config_data_UI, homingState=False, startSanding=False, scan=True)
            return jsonify({'status': 'success', 'message':'Table scan started'})
    
    return jsonify({'status': 'error', 'message': 'Invalid action provided'}), 400

############################################################################################
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5100, debug=True) # 0.0.0.0 is used to run on all available IPs
    # app.run(host='127.0.0.1', port=5100, debug=False, use_reloader=False)
