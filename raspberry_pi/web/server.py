"""
Web Server for Remote Control
=============================

Flask-based web interface for controlling the quadruped robot
from a phone or computer browser.
"""

from flask import Flask, render_template_string, jsonify, request
from flask_socketio import SocketIO
import threading
from typing import Optional, Callable
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import WEB_HOST, WEB_PORT


# HTML template for control interface
CONTROL_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
    <title>Quadruped Control</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        
        body {
            font-family: 'SF Mono', 'Fira Code', monospace;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 50%, #0f3460 100%);
            min-height: 100vh;
            color: #e8e8e8;
            overflow: hidden;
        }
        
        .container {
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: space-between;
            height: 100vh;
            padding: 20px;
        }
        
        h1 {
            font-size: 1.5rem;
            color: #00ff88;
            text-shadow: 0 0 20px rgba(0, 255, 136, 0.5);
            margin-bottom: 10px;
        }
        
        .status {
            display: flex;
            gap: 20px;
            font-size: 0.9rem;
            color: #888;
        }
        
        .status-item {
            display: flex;
            align-items: center;
            gap: 5px;
        }
        
        .status-dot {
            width: 10px;
            height: 10px;
            border-radius: 50%;
            background: #ff4444;
        }
        
        .status-dot.connected {
            background: #00ff88;
            box-shadow: 0 0 10px rgba(0, 255, 136, 0.5);
        }
        
        .controls {
            display: grid;
            grid-template-columns: repeat(3, 80px);
            grid-template-rows: repeat(3, 80px);
            gap: 10px;
        }
        
        .btn {
            width: 80px;
            height: 80px;
            border: 2px solid #00ff88;
            background: rgba(0, 255, 136, 0.1);
            color: #00ff88;
            font-size: 1.5rem;
            border-radius: 15px;
            cursor: pointer;
            transition: all 0.2s;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        
        .btn:active, .btn.active {
            background: rgba(0, 255, 136, 0.3);
            transform: scale(0.95);
            box-shadow: 0 0 20px rgba(0, 255, 136, 0.5);
        }
        
        .btn.center {
            background: rgba(255, 68, 68, 0.1);
            border-color: #ff4444;
            color: #ff4444;
        }
        
        .btn.center:active {
            background: rgba(255, 68, 68, 0.3);
            box-shadow: 0 0 20px rgba(255, 68, 68, 0.5);
        }
        
        .gait-selector {
            display: flex;
            gap: 10px;
        }
        
        .gait-btn {
            padding: 15px 25px;
            border: 2px solid #4488ff;
            background: rgba(68, 136, 255, 0.1);
            color: #4488ff;
            font-size: 1rem;
            font-family: inherit;
            border-radius: 10px;
            cursor: pointer;
            transition: all 0.2s;
        }
        
        .gait-btn:active, .gait-btn.selected {
            background: rgba(68, 136, 255, 0.3);
            box-shadow: 0 0 15px rgba(68, 136, 255, 0.5);
        }
        
        .telemetry {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 15px;
            width: 100%;
            max-width: 400px;
        }
        
        .telemetry-item {
            text-align: center;
            padding: 10px;
            background: rgba(255, 255, 255, 0.05);
            border-radius: 10px;
        }
        
        .telemetry-label {
            font-size: 0.8rem;
            color: #666;
            margin-bottom: 5px;
        }
        
        .telemetry-value {
            font-size: 1.2rem;
            color: #00ff88;
        }
    </style>
</head>
<body>
    <div class="container">
        <div>
            <h1>🤖 QUADRUPED CONTROL</h1>
            <div class="status">
                <div class="status-item">
                    <div class="status-dot" id="ws-status"></div>
                    <span id="connection-text">Disconnected</span>
                </div>
                <div class="status-item">
                    <span id="gait-status">Stand</span>
                </div>
            </div>
        </div>
        
        <div class="controls">
            <div></div>
            <button class="btn" id="btn-forward" data-cmd="forward">↑</button>
            <div></div>
            <button class="btn" id="btn-left" data-cmd="left">↶</button>
            <button class="btn center" id="btn-stop" data-cmd="stop">■</button>
            <button class="btn" id="btn-right" data-cmd="right">↷</button>
            <div></div>
            <button class="btn" id="btn-back" data-cmd="back">↓</button>
            <div></div>
        </div>
        
        <div class="gait-selector">
            <button class="gait-btn selected" data-gait="stand">Stand</button>
            <button class="gait-btn" data-gait="trot">Trot</button>
            <button class="gait-btn" data-gait="crawl">Crawl</button>
        </div>
        
        <div class="telemetry">
            <div class="telemetry-item">
                <div class="telemetry-label">ROLL</div>
                <div class="telemetry-value" id="roll">0.0°</div>
            </div>
            <div class="telemetry-item">
                <div class="telemetry-label">PITCH</div>
                <div class="telemetry-value" id="pitch">0.0°</div>
            </div>
            <div class="telemetry-item">
                <div class="telemetry-label">YAW</div>
                <div class="telemetry-value" id="yaw">0.0°</div>
            </div>
        </div>
    </div>
    
    <script src="https://cdn.socket.io/4.6.0/socket.io.min.js"></script>
    <script>
        const socket = io();
        let currentGait = 'stand';
        
        // Connection status
        socket.on('connect', () => {
            document.getElementById('ws-status').classList.add('connected');
            document.getElementById('connection-text').textContent = 'Connected';
        });
        
        socket.on('disconnect', () => {
            document.getElementById('ws-status').classList.remove('connected');
            document.getElementById('connection-text').textContent = 'Disconnected';
        });
        
        // Telemetry updates
        socket.on('telemetry', (data) => {
            document.getElementById('roll').textContent = data.roll.toFixed(1) + '°';
            document.getElementById('pitch').textContent = data.pitch.toFixed(1) + '°';
            document.getElementById('yaw').textContent = data.yaw.toFixed(1) + '°';
        });
        
        // Movement buttons
        document.querySelectorAll('.btn').forEach(btn => {
            const cmd = btn.dataset.cmd;
            
            // Touch/mouse down
            const startHandler = (e) => {
                e.preventDefault();
                btn.classList.add('active');
                socket.emit('command', { type: 'move', direction: cmd });
            };
            
            // Touch/mouse up
            const endHandler = (e) => {
                e.preventDefault();
                btn.classList.remove('active');
                socket.emit('command', { type: 'move', direction: 'stop' });
            };
            
            btn.addEventListener('mousedown', startHandler);
            btn.addEventListener('mouseup', endHandler);
            btn.addEventListener('mouseleave', endHandler);
            btn.addEventListener('touchstart', startHandler);
            btn.addEventListener('touchend', endHandler);
        });
        
        // Gait buttons
        document.querySelectorAll('.gait-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                document.querySelectorAll('.gait-btn').forEach(b => b.classList.remove('selected'));
                btn.classList.add('selected');
                currentGait = btn.dataset.gait;
                document.getElementById('gait-status').textContent = 
                    currentGait.charAt(0).toUpperCase() + currentGait.slice(1);
                socket.emit('command', { type: 'gait', gait: currentGait });
            });
        });
    </script>
</body>
</html>
"""


class WebServer:
    """
    Web server for remote control of the quadruped.
    """
    
    def __init__(self, command_callback: Optional[Callable] = None):
        """
        Initialize web server.
        
        Args:
            command_callback: Function to call when command received
        """
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = os.environ.get('QUADRUPED_SECRET_KEY', 'dev-only-change-in-production')
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        
        self.command_callback = command_callback
        self.telemetry = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        
        self._setup_routes()
        self._running = False
        self._thread: Optional[threading.Thread] = None
    
    def _setup_routes(self):
        """Set up Flask routes and SocketIO handlers."""
        
        @self.app.route('/')
        def index():
            return render_template_string(CONTROL_PAGE)
        
        @self.app.route('/api/status')
        def status():
            return jsonify({
                "status": "running",
                "telemetry": self.telemetry
            })
        
        @self.socketio.on('connect')
        def handle_connect():
            print("[WebServer] Client connected")
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            print("[WebServer] Client disconnected")
        
        @self.socketio.on('command')
        def handle_command(data):
            if self.command_callback:
                self.command_callback(data)
    
    def update_telemetry(self, roll: float, pitch: float, yaw: float):
        """
        Update telemetry data and broadcast to clients.
        
        Args:
            roll, pitch, yaw: IMU orientation angles
        """
        self.telemetry = {"roll": roll, "pitch": pitch, "yaw": yaw}
        self.socketio.emit('telemetry', self.telemetry)
    
    def start(self, host: str = WEB_HOST, port: int = WEB_PORT):
        """
        Start the web server in a background thread.
        
        Args:
            host: Host to bind to
            port: Port to listen on
        """
        if self._running:
            return
        
        self._running = True
        self._thread = threading.Thread(
            target=lambda: self.socketio.run(
                self.app, 
                host=host, 
                port=port, 
                debug=False,
                use_reloader=False,
                allow_unsafe_werkzeug=True
            ),
            daemon=True
        )
        self._thread.start()
        print(f"[WebServer] Started at http://{host}:{port}")
    
    def stop(self):
        """Stop the web server."""
        self._running = False
        print("[WebServer] Stopped")


# ============== TEST CODE ==============

if __name__ == "__main__":
    def handle_command(data):
        print(f"Command received: {data}")
    
    server = WebServer(command_callback=handle_command)
    server.start()
    
    print("Web server running. Open http://localhost:5000 in browser.")
    print("Press Ctrl+C to stop.")
    
    try:
        import time
        while True:
            # Simulate telemetry updates
            import random
            server.update_telemetry(
                roll=random.uniform(-10, 10),
                pitch=random.uniform(-10, 10),
                yaw=random.uniform(-180, 180)
            )
            time.sleep(0.1)
    except KeyboardInterrupt:
        server.stop()
