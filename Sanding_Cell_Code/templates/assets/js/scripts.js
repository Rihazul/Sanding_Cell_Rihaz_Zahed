let modalData = {};
const socket = io();


function submitConfigurations() {
    const table = selectedTable; // Selected table (A, B, or A/B)
    const doors = ['D1', 'D2', 'D3', 'D4']; // Define all doors


    // Close the modal after submission
    var modalElement = document.getElementById('confirmationModal');
    var confirmationModal = bootstrap.Modal.getInstance(modalElement);
    if (confirmationModal) {
        confirmationModal.hide(); // Close the modal after submission
    }
    sendModalData()
}
// Track if the robot currently “has” Tool 1
//let tool1Picked = false;

// Toggle function
//function toggleTool1() {
// If tool not picked yet, call 'pick'
// if (!tool1Picked) {
// sendToolAction(3, 'pick')
//   .then(() => {
// Update the button text
//  document.getElementById('tool1ToggleBtn').innerText = 'Keep Tool 1';
//  tool1Picked = true;
// })
// .catch(err => console.error(err));
// } 
//else {
// Tool is already picked, so let's keep it
// sendToolAction(3, 'keep')
// .then(() => {
//document.getElementById('tool1ToggleBtn').innerText = 'Pick Tool 1';
//tool1Picked = false;
//})
// .catch(err => console.error(err));
//}
//}
// Pick Tool 1 function
function pickTool3() {
    sendToolAction(3, 'pick')
        .then(() => {
            console.log('Tool 1 picked successfully');
        })
        .catch(err => console.error('Error picking Tool 1:', err));
}

// Keep Tool 1 function
function keepTool3() {
    sendToolAction(3, 'keep')
        .then(() => {
            console.log('Tool 1 kept successfully');
        })
        .catch(err => console.error('Error keeping Tool 1:', err));
}

// Helper that calls our new /tool_toggle endpoint
function sendToolAction(toolNumber, action) {
    return fetch('/tool_toggle', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ toolNumber, action })
    })
        .then(response => response.json())
        .then(data => {
            if (data.status !== 'success') {
                // optional: show an alert or handle error
                console.warn('Tool action error:', data);
            }
            return data;
        });
}

// Pick Tool 2 function
function pickTool2() {
    sendTool2Action(2, 'pick')
        .then(() => {
            console.log('Tool 2 picked successfully');
        })
        .catch(err => console.error('Error picking Tool 2:', err));
}

// Keep Tool 2 function
function keepTool2() {
    sendTool2Action(2, 'keep')
        .then(() => {
            console.log('Tool 2 kept successfully');
        })
        .catch(err => console.error('Error keeping Tool 2:', err));
}


// Helper for Tool 2
function sendTool2Action(toolNumber, action) {
    return fetch('/tool_toggle2', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ toolNumber, action })
    })
        .then(response => response.json())
        .then(data => {
            if (data.status !== 'success') {
                console.warn('Tool 2 action error:', data);
            }
            return data;
        });
}

// Pick Tool 1 function
function pickTool1() {
    sendTool1Action(1, 'pick')
        .then(() => {
            console.log('Tool 1 picked successfully');
        })
        .catch(err => console.error('Error picking Tool 1:', err));
}

// Keep Tool 1 function
function keepTool1() {
    sendTool1Action(1, 'keep')
        .then(() => {
            console.log('Tool 1 kept successfully');
        })
        .catch(err => console.error('Error keeping Tool 1:', err));
}


// Helper for Tool 1
function sendTool1Action(toolNumber, action) {
    return fetch('/tool_toggle1', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ toolNumber, action })
    })
        .then(response => response.json())
        .then(data => {
            if (data.status !== 'success') {
                console.warn('Tool 1 action error:', data);
            }
            return data;
        });
}


let selectedModelA = null; // To store the selected model for Table A
let selectedModelB = null; // To store the selected model for Table B

function selectModel(table, model) {
    if (table === 'A') {
        selectedModelA = null; // Reset model for Table A
        selectedModelA = model;
    } else if (table === 'B') {
        selectedModelA = null; // Reset model for Table A
        selectedModelB = model;
    }
}

let selectedTable = 'A'; // Default to Table A

function handleStartButtonClick() {
    // Ensure the selected table is properly validated
    if (!selectedTable) {
        alert('Please select a table (Table A, Table B, or Table A/B) before starting.');
        return;
    }

    // Validate Model Selection
    if (selectedTable === 'A' && !selectedModelA) {
        alert('Please select a model for Table A before starting.');
        return;
    }

    if (selectedTable === 'B') {
        alert('Please Upload the 3D file for Table B before starting.');
        return;
    }

    // Helper function to get cycle and force values for a given prefix (e.g., 'frameA')
    function getCycleForce(prefix) {
        const cycle = parseInt(document.getElementById(prefix + 'Input').value) || 0;
        const force = parseInt(document.getElementById(prefix + 'ForceInput').value) || 0;
        return { cycle, force };
    }

    // Construct the payload object
    const payload = {};

    if (selectedTable === 'A') {
        payload['TableA'] = {
            model: selectedModelA,
            frame: getCycleForce('frameA'),
            pocketzigzag: getCycleForce('pocketZigA'),
            pocketsquare: getCycleForce('pocketSQA'),
            '3D': getCycleForce('threeDA'),
            edgeInside: getCycleForce('inedgeA'),
            edgeOutside: getCycleForce('outedgeA'),
            side: getCycleForce('sideA')
        };
    }


    //no longer needed as 3d file is used
    if (selectedTable === 'B') {
        payload['TableB'] = {
            model: selectedModelB,
            frame: getCycleForce('frameB'),
            pocketzigzag: getCycleForce('pocketZigB'),
            pocketsquare: getCycleForce('pocketSQB'),
            '3D': getCycleForce('threeDB'),
            edgeInside: getCycleForce('inedgeB'),
            edgeOutside: getCycleForce('outedgeB'),
            side: getCycleForce('sideB')
        };
    }



    console.log("Payload to send:", JSON.stringify(payload, null, 2));

    // Send POST request to the API endpoint
    fetch('/start_process', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify(payload)
    })
        .then(response => {
            if (!response.ok) {
                throw new Error('Network response was not ok');
            }
            return response.json();
        })
        .then(data => {
            console.log('Success:', data);
            // Optionally show success message or update UI
        })
        .catch(error => {
            console.error('Error:', error);
            alert('Failed to start process: ' + error.message);
        });
}


// Function to enable or disable tables based on selection
function selectTable(table) {
    selectedTable = table;
    const tableAConfig = document.getElementById('tableAConfig');
    const tableBConfig = document.getElementById('tableBConfig');
    const btnTableA = document.getElementById('btnTableA');
    const btnTableB = document.getElementById('btnTableB');

    // Reset button states
    btnTableA.classList.remove('btn-selected');
    btnTableB.classList.remove('btn-selected');

    // Handle table selection
    if (table === 'A') {
        tableAConfig.classList.remove('column-disabled');
        tableBConfig.classList.add('column-disabled');
        btnTableA.classList.add('btn-selected');
    } else if (table === 'B') {
        tableAConfig.classList.add('column-disabled');
        tableBConfig.classList.remove('column-disabled');
        btnTableB.classList.add('btn-selected');
    } else if (table === 'A/B') {
        tableAConfig.classList.remove('column-disabled');
        tableBConfig.classList.remove('column-disabled');
    }
}

// Synchronize input and slider for all configurations
function syncInputSlider(input, slider) {
    // When the input field changes, update the slider and the track fill
    input.addEventListener('input', () => {
        slider.value = input.value;
        updateSliderProgress(slider); // Update the filled part of the track
    });

    // When the slider changes, update the input field and the track fill
    slider.addEventListener('input', () => {
        input.value = slider.value;
        updateSliderProgress(slider); // Update the filled part of the track
    });

    // Ensure that the initial state is set correctly
    updateSliderProgress(slider);
}

function updateModelImage(table) {

    if (table == 'A') {
        select = document.getElementById('modelSelectA');
        modelImage = document.getElementById('modelImageA');
        selectedTable = 'A';
    } else if (table == 'B') {
        select = document.getElementById('modelSelectB');
        modelImage = document.getElementById('modelImageB');
        selectedTable = 'B';
    } else {
        return; // Invalid table selection
    }
    // const modelImage = document.getElementById('modelImage');

    // Get the selected value
    const selectedModel = select.value;

    selectModel(table, selectedModel); // Update the selected model

    // Show the image container
    modelImage.style.display = 'block';

    // Set the appropriate image based on selection
    switch (selectedModel) {
        case 'modelA':
            if (selectedTable == 'A'){
                modelImage.src = "{{ url_for('static', filename='table_1/model1.jpg') }}";
            } else {
                modelImage.src = "{{ url_for('static', filename='table_2/model1.jpeg') }}";
            }
            break;
        case 'modelB':
            if (selectedTable == 'A'){
                modelImage.src = "{{ url_for('static', filename='table_1/model2.jpg') }}";
            } else {
                modelImage.src = "{{ url_for('static', filename='table_2/model2.jpeg') }}";
            }
            break;
        case 'modelC':
            if (selectedTable == 'A'){
                modelImage.src = "{{ url_for('static', filename='table_1/model3.png') }}";
            } else {
                modelImage.src = "{{ url_for('static', filename='table_2/model3.jpeg') }}";
            }
            break;
        case 'modelD':
            if (selectedTable == 'A'){
                modelImage.src = "{{ url_for('static', filename='table_1/model4.jpg') }}";
            } else {
                // modelImage.src = "{{ url_for('static', filename='table_2/model4.jpeg') }}";
            }
            break;
        case 'modelE':
            if (selectedTable == 'A'){
                modelImage.src = "{{ url_for('static', filename='table_1/model5.jpg') }}";
            } else {
                modelImage.src = "{{ url_for('static', filename='table_2/model5.jpeg') }}";
            }
            break;
        default:
            modelImage.style.display = 'none';
            modelImage.src = '';
    }
}



syncInputSlider(document.getElementById('robotSpeedInput'), document.getElementById('robotSpeedSlider'));
syncInputSlider(document.getElementById('inverseOverlappingInput'), document.getElementById('inverseOverlappingSlider'));
// syncInputSlider(document.getElementById('frameAInput'), document.getElementById('frameASlider'));
// syncInputSlider(document.getElementById('sideBInput'), document.getElementById('sideBSlider'));
// syncInputSlider(document.getElementById('inedgeAInput'), document.getElementById('inedgeASlider'));
// syncInputSlider(document.getElementById('outedgeAInput'), document.getElementById('outedgeASlider'));
// syncInputSlider(document.getElementById('pocketZigAInput'), document.getElementById('pocketZigASlider'));
// syncInputSlider(document.getElementById('pocketSQAInput'), document.getElementById('pocketSQASlider'));
// syncInputSlider(document.getElementById('threeDBInput'), document.getElementById('threeDBSlider'));
// syncInputSlider(document.getElementById('frameBInput'), document.getElementById('frameBSlider'));
// syncInputSlider(document.getElementById('sideBInput'), document.getElementById('sideBSlider'));
// syncInputSlider(document.getElementById('inedgeBInput'), document.getElementById('inedgeBSlider'));
// syncInputSlider(document.getElementById('outedgeBInput'), document.getElementById('outedgeBSlider'));
// syncInputSlider(document.getElementById('pocketZigBInput'), document.getElementById('pocketZigBSlider'));
// syncInputSlider(document.getElementById('pocketSQBInput'), document.getElementById('pocketSQBSlider'));
// syncInputSlider(document.getElementById('threeDBInput'), document.getElementById('threeDBSlider'));

// Initialize tooltips
document.querySelectorAll('[data-bs-toggle="tooltip"]').forEach(function (tooltipTriggerEl) {
    new bootstrap.Tooltip(tooltipTriggerEl);
});

// Function to update the slider's fill
function updateSliderProgress(slider) {
    const value = (slider.value - slider.min) / (slider.max - slider.min) * 100;
    slider.style.setProperty('--value', value + '%');
}

// Apply the update function to each slider
document.querySelectorAll('input[type="range"]').forEach(slider => {
    updateSliderProgress(slider); // Set initial value
    slider.addEventListener('input', () => updateSliderProgress(slider)); // Update on input
});

// Store previous data for comparison
let previousData = null;

// Function to update the UI if new data is received
function updateUI(data) {
    document.getElementById('length-value').textContent = data.length;
    document.getElementById('width-value').textContent = data.width;
    document.getElementById('breadth-value').textContent = data.breadth;

    updateProgressBar('frame-progress-bar', data.frame_progress.numerator, data.frame_progress.denominator, 'frame-value');
    updateProgressBar('side-progress-bar', data.side_progress.numerator, data.side_progress.denominator, 'side-value');
    updateProgressBar('pocket-progress-bar', data.pocket_progress.numerator, data.pocket_progress.denominator, 'pocket-value');
    updateProgressBar('edge-progress-bar', data.edge_progress.numerator, data.edge_progress.denominator, 'edge-value');
    updateProgressBar('threeD-progress-bar', data.threeD_progress.numerator, data.threeD_progress.denominator, 'threeD-value');
}

// Function to fetch data and only update UI if data has changed
// function fetchDataAndUpdate() {
//     const tableSelect = document.getElementById('tableSelect').value;
//     const doorSelect = document.getElementById('doorSelect').value;

//     fetch(`/get_data?table=${tableSelect}&door=${doorSelect}`)
//         .then(response => response.json())
//         .then(data => {
//             if (JSON.stringify(data) !== JSON.stringify(previousData)) {
//                 updateUI(data); // Update the UI if data has changed
//                 previousData = data; // Store new data for future comparison
//             }
//         })
//         .catch(error => console.error('Error fetching data:', error));
// }

// // Initialize data fetching and periodic sync without using window.onload
// fetchDataAndUpdate(); // Initial data fetch on script execution
// setInterval(fetchDataAndUpdate, 5000); // Sync every 5 seconds


// Function to update the width of progress bars based on the numerator and denominator
// function updateProgressBar(id, numerator, denominator, valueId) {
//     const progressBar = document.getElementById(id);
//     const percentage = (numerator / denominator) * 100;

//     // Check for denominator to prevent division by zero
//     if (denominator === 0) {
//         progressBar.style.width = '0%';
//         document.getElementById(valueId).textContent = `0/0`;
//     } else {
//         progressBar.style.width = `${percentage}%`;
//         document.getElementById(valueId).textContent = `${numerator}/${denominator}`;
//     }

//     // For accessibility, update the aria-valuenow attribute
//     progressBar.setAttribute('aria-valuenow', percentage);

//     // Debugging: Log the values to ensure they are updated correctly
//     console.log(`Updated progress bar [${id}] to ${percentage}% (${numerator}/${denominator})`);
// }

// Add event listeners to dropdowns to trigger data fetch and update
// document.getElementById('tableSelect').addEventListener('change', fetchDataAndUpdate);
// document.getElementById('doorSelect').addEventListener('change', fetchDataAndUpdate);
const originalUploadAreaHTML = document.getElementById('uploadArea').innerHTML;

function uploadFile() {
    // Show the upload modal
    const modal = new bootstrap.Modal(document.getElementById('fileUploadModal'));
    modal.show();
    document.getElementById('uploadArea').innerHTML = originalUploadAreaHTML;
    const uploadArea = document.getElementById('uploadArea');
    const fileInput = document.getElementById('fileInput');

    // Remove any existing event listeners
    uploadArea.removeEventListener('click', handleUploadAreaClick);
    fileInput.removeEventListener('change', handleFileSelect);
    uploadArea.removeEventListener('dragover', handleDragOver);
    uploadArea.removeEventListener('dragleave', handleDragLeave);
    uploadArea.removeEventListener('drop', handleDrop);

    // Attach the event listeners
    uploadArea.addEventListener('click', handleUploadAreaClick);
    fileInput.addEventListener('change', handleFileSelect);
    uploadArea.addEventListener('dragover', handleDragOver);
    uploadArea.addEventListener('dragleave', handleDragLeave);
    uploadArea.addEventListener('drop', handleDrop);

}

function handleUploadAreaClick() {
    document.getElementById('fileInput').click();
}

function handleFileSelect(e) {
    const files = e.target.files;
    handleFiles(files);
}

function handleDragOver(e) {
    e.preventDefault();
    document.getElementById('uploadArea').classList.add('dragover');
}

function handleDragLeave() {
    document.getElementById('uploadArea').classList.remove('dragover');
}

function handleDrop(e) {
    e.preventDefault();
    document.getElementById('uploadArea').classList.remove('dragover');
    const files = e.dataTransfer.files;
    handleFiles(files);
}

function handleFiles(files) {
    if (files.length === 0) return;

    const file = files[0];
    const allowedTypes = ['.stp'];
    const fileExtension = '.' + file.name.split('.').pop().toLowerCase();

    if (!allowedTypes.includes(fileExtension)) {
        alert('Please select a valid 3D file format (.stp)');
        return;
    }

    // Create FormData and append file
    const formData = new FormData();
    formData.append('file', file);

    // Show loading state
    const uploadArea = document.getElementById('uploadArea');
    uploadArea.innerHTML = '<div class="spinner-border text-danger" role="status"><span class="visually-hidden">Loading...</span></div>';

    // Send file to server
    fetch('/upload_3d_file', {
        method: 'POST',
        body: formData
    })
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                // var modal = new bootstrap.Modal(document.getElementById('fileUploadModal'));
                // modal.hide();
                // Show success message
                uploadArea.innerHTML = '<div class="text-success"><i class="bi bi-check-circle" style="font-size: 3rem;"></i><p>File uploaded successfully!</p></div>';
                document.getElementById('start-Button').disabled = false; // Enable the start button                
            } else {
                throw new Error(data.message || 'Upload failed');
            }
        })
        .catch(error => {
            uploadArea.innerHTML = `<div class="text-danger"><i class="bi bi-x-circle" style="font-size: 3rem;"></i><p>${error.message}</p></div>`;
        });
}

function getSelectedDoors(containerId) {
    const container = document.getElementById(containerId);
    if (!container) {
        console.warn(`Container with ID ${containerId} not found.`);
        return [];
    }
    // Select only boxes that are selected AND have the 'number' data-box-type
    const selectedBoxes = container.querySelectorAll('.frame-box.selected[data-box-type="number"]');
    // Convert NodeList to Array and map over to get the text content of each box
    const doorNumbers = Array.from(selectedBoxes).map(box => parseInt(box.textContent, 10));
    return doorNumbers;
}


// Function to handle the start button click
function startSanding() {
    // Populate the confirmation modal with selected values
    const confirmModelA = selectedModelA || 'None';
    const confirmModelB = selectedModelB || 'None';

    if (selectedTable == 'A') {
        payloadJson =
        {
            "TableA": {
                "model": confirmModelA,
                "frame": {
                    "cycle": document.getElementById('frameAInput').value,
                    "force": document.getElementById('frameAForceInput').value,
                    //need to add selected door in and array such as ["1", "2", "3", "4"] which ever door are selected for frame
                    "doors": getSelectedDoors('frameA_FrameBoxes')
                },
                "pocketzigzag": {
                    "cycle": document.getElementById('pocketZigAInput').value,
                    "force": document.getElementById('pocketZigAForceInput').value,
                    //need to add selected door in and array such as ["1", "2", "3", "4"] which ever door are selected for pocketzigzag
                    "doors": getSelectedDoors('frameA_PocketZigBoxes')
                },
                "pocketsquare": {
                    "cycle": document.getElementById('pocketSQAInput').value,
                    "force": document.getElementById('pocketSQAForceInput').value,
                    //need to add selected door in and array such as ["1", "2", "3", "4"] which ever door are selected for pocketsquare
                    "doors": getSelectedDoors('frameA_PocketSQBoxes')
                },
                "3D": {
                    "cycle": document.getElementById('threeDAInput').value,
                    "force": document.getElementById('threeDAForceInput').value,
                    //need to add selected door in and array such as ["1", "2", "3", "4"] which ever door are selected for 3D
                    "doors": getSelectedDoors('frameA_3DBoxes')
                },
                "edgeInside": {
                    "cycle": document.getElementById('inedgeAInput').value,
                    "force": document.getElementById('inedgeAForceInput').value,
                    //need to add selected door in and array such as ["1", "2", "3", "4"] which ever door are selected for edgeInside
                    "doors": getSelectedDoors('frameA_InEdgeBoxes')
                },
                "edgeOutside": {
                    "cycle": document.getElementById('outedgeAInput').value,
                    "force": document.getElementById('outedgeAForceInput').value,
                    //need to add selected door in and array such as ["1", "2", "3", "4"] which ever door are selected for edgeOutside
                    "doors": getSelectedDoors('frameA_OutEdgeBoxes')
                },
                "side": {
                    "cycle": document.getElementById('sideAInput').value,
                    "force": document.getElementById('sideAForceInput').value,
                    //need to add selected door in and array such as ["1", "2", "3", "4"] which ever door are selected for side
                    "doors": getSelectedDoors('frameA_SideBoxes')
                }
            },
            "robotSpeed": (document.getElementById('robotSpeedInput').value / 100).toString(),
            "inverseOverlapping": document.getElementById('inverseOverlappingInput').value
        }
        return fetch('/start_TableA_process', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify(payloadJson)
        })
            .then(response => response.json())
            .then(data => {
                console.log('Success:', data);
                console.log("the data.success is !", data.success);
                if (data.success) {
                    Swal.fire({
                        title: 'Success!',
                        text: `${data.status}`,
                        icon: 'success',
                        timer: 2000, // Automatically close after 2 seconds
                        showConfirmButton: false // Hide the "OK" button
                    });
                } else if (!data.success && data.process == 'cancelled') {
                    Swal.fire({
                        title: 'Cancelled!',
                        text: `${data.status}`,
                        icon: 'warning',
                        timer: 2000, // Automatically close after 2 seconds
                        showConfirmButton: false // Hide the "OK" button
                    });
                } else {
                    alert(`Failed to startProcess: ${data.message}`);
                }
            })
    }
    else if (selectedTable == 'B') {
        payloadJson =
        {
            "TableB": {
                "model": confirmModelB,
                "frame": {
                    "cycle": document.getElementById('frameBInput').value,
                    "force": document.getElementById('frameBForceInput').value
                },
                "pocketzigzag": {
                    "cycle": document.getElementById('pocketZigBInput').value,
                    "force": document.getElementById('pocketZigBForceInput').value
                },
                "pocketsquare": {
                    "cycle": document.getElementById('pocketSQBInput').value,
                    "force": document.getElementById('pocketSQBForceInput').value
                },
                "3D": {
                    "cycle": document.getElementById('threeDBInput').value,
                    "force": document.getElementById('threeDBForceInput').value
                },
                "edgeInside": {
                    "cycle": document.getElementById('inedgeBInput').value,
                    "force": document.getElementById('inedgeBForceInput').value
                },
                "edgeOutside": {
                    "cycle": document.getElementById('outedgeBInput').value,
                    "force": document.getElementById('outedgeBForceInput').value
                },
                "side": {
                    "cycle": document.getElementById('sideBInput').value,
                    "force": document.getElementById('sideBForceInput').value
                }
            },
            "robotSpeed": (document.getElementById('robotSpeedInput').value / 100).toString(),
            "inverseOverlapping": document.getElementById('inverseOverlappingInput').value
        }
        return fetch('/start_TableB_process', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify(payloadJson)
        })
            .then(response => response.json())
            .then(data => {
                console.log('Success:', data);

                // First close the upload modal
                const uploadModal = bootstrap.Modal.getInstance(document.getElementById('fileUploadModal'));
                if (uploadModal) {
                    uploadModal.hide();
                }
                console.log("the data.success is !", data.success);
                if (data.success) {
                    Swal.fire({
                        title: 'Success!',
                        text: `${data.status}`,
                        icon: 'success',
                        timer: 2000, // Automatically close after 2 seconds
                        showConfirmButton: false // Hide the "OK" button
                    });
                } else if (!data.success && data.process == 'cancelled') {
                    Swal.fire({
                        title: 'Cancelled!',
                        text: `${data.status}`,
                        icon: 'warning',
                        timer: 2000, // Automatically close after 2 seconds
                        showConfirmButton: false // Hide the "OK" button
                    });
                } else {
                    alert(`Failed to startProcess: ${data.message}`);
                }
            })
    } else {
        alert('Please select a table (Table A, Table B, or Table A/B) before starting.');
        return;
    }

    // //call the api and send the payload json
    // fetch('/start_TableB_process', {
    //     method: 'POST',
    //     headers: {
    //         'Content-Type': 'application/json'
    //     },
    //     body: JSON.stringify(payloadJson)
    // })
    // .then(response => response.json())
    // .then(data => {
    //     console.log('Success:', data);

    //     // First close the upload modal
    //     const uploadModal = bootstrap.Modal.getInstance(document.getElementById('fileUploadModal'));
    //     if (uploadModal) {
    //         uploadModal.hide();
    //     }
    //     console.log("the data.success is !", data.success);
    //     if (data.success) {
    //         Swal.fire({
    //             title: 'Success!',
    //             text: `${data.status}`,
    //             icon: 'success',
    //             timer: 2000, // Automatically close after 2 seconds
    //             showConfirmButton: false // Hide the "OK" button
    //         });
    //     } else if(!data.success && data.process == 'cancelled') {
    //         Swal.fire({
    //             title: 'Cancelled!',
    //             text: `${data.status}`,
    //             icon: 'warning',
    //             timer: 2000, // Automatically close after 2 seconds
    //             showConfirmButton: false // Hide the "OK" button
    //         });
    //     } else {
    //         alert(`Failed to startProcess: ${data.message}`);
    //     }
    // })
}



// Update the robot GIF and process status
function startProcess(action) {
    fetch('/action', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({ action: action })
    })
        .then(response => response.json())
        .then(data => {
            console.log(data)
            // if (action !== 'start') {
            //     pass
            //     // alert(data.message);  // Show response message for actions other than "start"
            // }
        })
        .catch(error => {
            console.error('Error:', error);
        });
}


// Function to handle sending modalData separately to /save_modal_data
function sendModalData() {
    console.log("Modal Data:", JSON.stringify(modalData, null, 2));

    fetch('/save_modal_data', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify(modalData)
    })
        .then(response => response.json())
        .then(data => {
            console.log('Modal data saved:', data.message);
        })
        .catch(error => {
            console.error('Error in /save_modal_data request:', error);
        });
}

function toggleState(tableId, event) {
    fetch(`/toggle_state/${tableId}`)
        .then(response => response.json())
        .then(data => {
            const button = event.target.closest('button');
            const span = button.querySelector('span');  // Target the inner span

            const currentText = span.textContent;
            const newState = data.newState.trim();  // e.g., "Open" or "Close"

            // Update the state text
            const updatedText = currentText.replace(/(Open|Close)$/, newState);
            span.textContent = updatedText;  // Update only the state part
        })
        .catch(error => console.error('Error:', error));
}

function fetchAndUpdateTableStates(tableIds) {
    tableIds.forEach(tableId => {
        fetch(`/get_state/${tableId}`)
            .then(response => response.json())
            .then(data => {
                const button = document.querySelector(`button[data-table-id="${tableId}"]`);
                if (button) {
                    const span = button.querySelector('span');  // Target the inner span
                    const currentText = span.textContent;
                    const newState = data.newState.trim();  // e.g., "Open" or "Close"
                    console.log("Reached Span!", newState)

                    // Update the state text
                    const updatedText = currentText.replace(/(Open|Close)$/, newState);
                    console.log("Reached Span2!", updatedText)
                    span.textContent = updatedText;  // Update only the state part
                }
            })
            .catch(error => console.error(`Error updating state for ${tableId}:`, error));
    });
}

function setupFrameBoxToggle(containerId) {
    const container = document.getElementById(containerId);
    if (!container) return; // Exit if container not found

    const allBoxes = Array.from(container.querySelectorAll('.frame-box'));
    const numberBoxes = allBoxes.filter(box => box.dataset.boxType === 'number');
    const ABox = allBoxes.find(box => box.dataset.boxType === 'A');

    container.addEventListener('click', function (event) {
        const clickedBox = event.target;
        if (!clickedBox.classList.contains('frame-box')) {
            return; // Not a frame box
        }

        if (clickedBox.dataset.boxType === 'A') {
            // Logic for 'A' box click
            const isSelected = clickedBox.classList.toggle('selected');
            numberBoxes.forEach(box => {
                if (isSelected) {
                    box.classList.add('selected');
                } else {
                    box.classList.remove('selected');
                }
            });
        } else if (clickedBox.dataset.boxType === 'number') {
            // Logic for number box click
            clickedBox.classList.toggle('selected');

            // Check if all number boxes are now selected
            const allNumbersSelected = numberBoxes.every(box => box.classList.contains('selected'));
            if (ABox) { // Ensure ABox exists
                if (allNumbersSelected) {
                    ABox.classList.add('selected');
                } else {
                    ABox.classList.remove('selected');
                }
            }
        }
    });
}

document.addEventListener('DOMContentLoaded', () => {
    //     const tableIds = ['tableAOpenClose', 'tableBOpenClose'];
    //     fetchAndUpdateTableStates(tableIds);
    //     setInterval(() => {
    //     fetch('/check_tool1_status')
    //         .then(resp => resp.json())
    //         .then(data => {
    //             console.log("Tool1 check:", data);
    //             // The actual blink logic is triggered server-side 
    //             // and sent to us via socket.io ('blink_circle_button').
    //         })
    //         .catch(err => console.error('Error checking tool1 status:', err));
    // }, 2000);
    //     setInterval(() => {
    //     fetch('/check_tool2_status')
    //         .then(resp => resp.json())
    //         .then(data => {
    //             console.log("Tool 2 check:", data);
    //         // The actual blinking is handled by the event 'blink_circle_button2'
    //         })
    //         .catch(err => console.error("Error checking Tool2 status:", err));
    // }, 2000);
    // setInterval(() => {
    //     fetch('/check_tool3_status')
    //         .then(resp => resp.json())
    //         .then(data => {
    //             console.log("Tool 3 check:", data);
    //         // The actual blinking is handled by the event 'blink_circle_button2'
    //         })
    //         .catch(err => console.error("Error checking Tool3 status:", err));
    // }, 2000);

    setupFrameBoxToggle('frameA_FrameBoxes');
    setupFrameBoxToggle('frameA_PocketZigBoxes');
    setupFrameBoxToggle('frameA_PocketSQBoxes');
    setupFrameBoxToggle('frameA_3DBoxes');
    setupFrameBoxToggle('frameA_InEdgeBoxes');
    setupFrameBoxToggle('frameA_OutEdgeBoxes');
    setupFrameBoxToggle('frameA_SideBoxes');

    const tableIds = ['tableAOpenClose', 'tableBOpenClose'];
    fetchAndUpdateTableStates(tableIds);


});

socket.on('flash_message', (data) => {
    console.log("Received flash message:", data);

    const flashContainer = document.getElementById('flashContainer');
    if (!flashContainer) {
        console.error("Flash container not found!");
        return;
    }

    // Clear the container to ensure only one message is displayed at a time
    flashContainer.innerHTML = '';

    // Create a new alert
    //const alert = document.createElement('div');
    //alert.className = `alert alert-success alert-dismissible fade show`;
    //alert.role = 'alert';
    // Create a new alert message
    const alert = document.createElement('div');
    alert.className = `alert alert-dismissible fade show`;
    alert.role = 'alert';

    // Apply conditional styling based on message type
    if (data.type === 'warning') {
        alert.style.color = 'red';
        alert.style.fontWeight = 'bold';
        alert.style.fontSize = '20px';
    } else {
        alert.classList.add('alert-success'); // Default styling for success messages
    }

    alert.innerHTML = `
${data.message}
<button type="button" class="btn-close" data-bs-dismiss="alert" aria-label="Close"></button>
`;

    // Append the alert to the container
    flashContainer.appendChild(alert);
});
socket.on('blink_circle_button', (data) => {
    const circleBtn = document.getElementById('circleBtn');
    if (data.shouldBlink) {
        // Add blinking animation
        circleBtn.classList.add('blinking');
    } else {
        // Remove blinking animation
        circleBtn.classList.remove('blinking');
    }
});
socket.on('blink_circle_button2', (data) => {
    const circleBtn2 = document.getElementById('circleBtn2');
    if (data.shouldBlink) {
        circleBtn2.classList.add('blinking');  // same blinking class or a different one
    } else {
        circleBtn2.classList.remove('blinking');
    }
});
socket.on('blink_circle_button3', (data) => {
    const circleBtn3 = document.getElementById('circleBtn3');
    if (data.shouldBlink) {
        circleBtn3.classList.add('blinking');  // same blinking class or a different one
    } else {
        circleBtn3.classList.remove('blinking');
    }
});