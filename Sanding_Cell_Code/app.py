import subprocess
import threading
import webview
import os
import signal
import sys

# Function to start Flask in a separate thread
def start_flask():
    flask_process = subprocess.Popen(['python', 'flask_app.py'])
    return flask_process

# Function to kill the Flask process when the window is closed
def close_flask_process(flask_process):
    try:
        # Kill the Flask process
        if flask_process.poll() is None:  # Check if the process is still running
            flask_process.terminate()  # Gracefully terminate the process
            flask_process.wait()  # Wait for the process to terminate
    except Exception as e:
        print(f"Error terminating Flask process: {e}")

# Function to create the PyWebView window and control the process
def create_window():
    # Start Flask in a separate thread and get the process handle
    # flask_process = start_flask()

    # Create the PyWebView window (Disable resizing)
    window = webview.create_window('Sanding App', 'http://localhost:5100', resizable=True, width=1680, height=1050)

    # Define the close action callback
    def on_window_closed():
        # Stop the Flask server (kill the process) when the window is closed
        close_flask_process(flask_process)

    # Start the webview and handle window close (this must run in the main thread)
    # webview.start(on_window_closed)
    webview.start()

if __name__ == '__main__':
    # Start Flask in a separate thread but ensure PyWebView runs in the main thread
    flask_thread = threading.Thread(target=start_flask)
    flask_thread.daemon = True  # Set to daemon so the Flask thread stops when the main thread exits
    # flask_thread.start()

    # Now run PyWebView on the main thread
    create_window()
