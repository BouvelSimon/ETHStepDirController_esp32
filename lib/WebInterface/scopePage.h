R"html(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Data vs Time Display</title>
    <style>
        body {
            font-family: 'Roboto', Arial, sans-serif;
            background-color: #0a0a0a;
            color: #e0e0e0;
            margin: 0;
            padding: 20px;
            min-height: 100vh;
            background-image: linear-gradient(45deg, #0a0a0a 25%, #0f0f0f 25%, #0f0f0f 50%, #0a0a0a 50%, #0a0a0a 75%, #0f0f0f 75%, #0f0f0f 100%);
            background-size: 40px 40px;
        }
        h1 {
            color: #b0b0b0;
            margin-bottom: 20px;
        }
        canvas {
            border: 1px solid #333;
            background-color: #f0f0f0;
            border-radius: 12px;
            box-shadow: 0 4px 15px rgba(0, 0, 0, 0.3);
        }
        #controls {
            margin: 20px 0;
            display: flex;
            flex-wrap: wrap;
            gap: 15px;
            align-items: center;
        }
        #latestValues {
            margin-top: 20px;
            font-family: Arial, sans-serif;
            background-color: #1a1a1a;
            padding: 15px;
            border-radius: 12px;
            box-shadow: 0 4px 15px rgba(0, 0, 0, 0.3);
        }
        #latestValues span {
            display: block;
            margin: 5px 0;
            color: #b0b0b0;
        }
        #displayContainer {
            display: flex;
            justify-content: space-between;
            flex-wrap: wrap;
        }
        label {
            font-weight: 500;
            color: #b0b0b0;
            font-size: 0.9em;
            letter-spacing: 0.3px;
        }
        input[type="number"], input[type="checkbox"] {
            background-color: #252525;
            border: 1px solid #333;
            border-radius: 6px;
            color: #e0e0e0;
            padding: 5px;
            font-size: 13px;
        }
        input[type="number"]:focus, input[type="checkbox"]:focus {
            outline: none;
            border-color: #4a4a4a;
            box-shadow: 0 0 0 2px rgba(74, 74, 74, 0.3);
        }
        @keyframes subtle-glow {
            0% { box-shadow: 0 4px 15px rgba(0, 0, 0, 0.3); }
            50% { box-shadow: 0 4px 20px rgba(30, 30, 30, 0.4); }
            100% { box-shadow: 0 4px 15px rgba(0, 0, 0, 0.3); }
        }
        #latestValues, canvas {
            animation: subtle-glow 5s infinite alternate;
        }
    </style>
</head>
<body>
    <h1>Data vs Time Display</h1>

    <div id="controls">
        <label for="refreshRate">Refresh Rate (ms):</label>
        <input type="number" id="refreshRate" value="100" min="20" step="20">

        <label for="timeWindow">Time Window (seconds):</label>
        <input type="number" id="timeWindow" value="30" min="1" step="1">

        <label><input type="checkbox" id="motorPulses" checked> Motor Pulses</label>
        <label><input type="checkbox" id="encoderPosition" checked> Encoder Position</label>
        <label><input type="checkbox" id="setPoint" checked> SetPoint</label>

        <label><input type="checkbox" id="freezeDisplay"> Freeze Display</label>
    </div>

    <div id="displayContainer">
        <canvas id="dataCanvas" width="800" height="400"></canvas>
        <div id="latestValues">
            <h3>Latest Values</h3>
            <span id="latestMotorPulses">Motor Pulses: N/A</span>
            <span id="latestEncoderPosition">Encoder Position: N/A</span>
            <span id="latestSetPoint">SetPoint: N/A</span>
        </div>
    </div>

    <script>
        const canvas = document.getElementById('dataCanvas');
        const ctx = canvas.getContext('2d');

        const motorPulsesCheckbox = document.getElementById('motorPulses');
        const encoderPositionCheckbox = document.getElementById('encoderPosition');
        const setPointCheckbox = document.getElementById('setPoint');
        const refreshRateInput = document.getElementById('refreshRate');
        const timeWindowInput = document.getElementById('timeWindow');
        const freezeDisplayCheckbox = document.getElementById('freezeDisplay');

        const latestMotorPulses = document.getElementById('latestMotorPulses');
        const latestEncoderPosition = document.getElementById('latestEncoderPosition');
        const latestSetPoint = document.getElementById('latestSetPoint');

        let motorPulsesData = [];
        let encoderPositionData = [];
        let setPointData = [];
        let timeData = [];
        let intervalId;

        function fetchJSONData() {
            return fetch('http://{{IP_ADDRESS}}/feedback', {
                method: 'GET',
                headers: {'Content-Type': 'application/json'},
                mode: 'cors' // Explicitly set CORS mode
            })
            .then(response => {
                if (!response.ok) {
                    throw new Error(`HTTP error! status: ${response.status}`);
                }
                return response.json();
            })
            .then(data => {
                if (data.pulses !== undefined && data.encoder !== undefined && data.timeStamp !== undefined && data.setPoint !== undefined) {
                    return {
                        motorPulses: data.pulses,
                        encoderPosition: data.encoder,
                        timeStamp: data.timeStamp,
                        setPoint: data.setPoint
                    };
                } else {
                    throw new Error('Invalid data structure received');
                }
            })
            .catch(error => {
                console.error('Error fetching data:', error);
                document.getElementById('errorLog').textContent = `Fetch error: ${error.message}`;
                return {
                    motorPulses: 0,
                    encoderPosition: 0,
                    timeStamp: Date.now() / 1000,
                    setPoint: 0
                };
            });
        }

        // Draw legend
        function drawLegend() {
            ctx.font = '12px Arial';
            ctx.fillStyle = 'red';
            ctx.fillText('Motor Pulses', 680, 30);
            ctx.fillStyle = 'green';
            ctx.fillText('Encoder Position', 680, 50);
            ctx.fillStyle = 'blue';
            ctx.fillText('SetPoint', 680, 70);
        }

        // Draw X and Y axis ticks and labels
        function drawAxes(maxTime, minTime, minY, maxY) {
            ctx.strokeStyle = 'black';
            ctx.beginPath();
            ctx.moveTo(50, 0);
            ctx.lineTo(50, canvas.height - 20); // Y-axis
            ctx.lineTo(canvas.width, canvas.height - 20); // X-axis
            ctx.stroke();

            // Y-axis label
            ctx.font = '14px Arial';
            ctx.fillStyle = 'black';
            ctx.save();
            ctx.translate(20, canvas.height / 2); // Rotate the canvas for Y-axis label
            ctx.rotate(-Math.PI / 2);
            ctx.fillText('Motor Steps', 0, 0);
            ctx.restore();

            // Y-axis ticks and labels
            const yTicks = 10;
            const yRange = maxY - minY;
            const yStep = (canvas.height - 20) / yTicks;
            for (let i = 0; i <= yTicks; i++) {
                const y = canvas.height - 20 - i * yStep;
                const value = minY + (i * (yRange / yTicks));
                ctx.fillStyle = 'black';
                ctx.fillText(value.toFixed(0), 15, y);
                ctx.beginPath();
                ctx.moveTo(50, y);
                ctx.lineTo(55, y); // Tick mark
                ctx.stroke();
            }

            // X-axis ticks and labels based on timeStamp (seconds)
            const xTicks = 5;
            const timeRange = maxTime - minTime;
            const xStep = (canvas.width - 60) / xTicks;
            for (let i = 0; i <= xTicks; i++) {
                const x = 50 + i * xStep;
                const timeLabel = minTime + i * (timeRange / xTicks);
                ctx.fillText(timeLabel.toFixed(2) + 's', x - 20, canvas.height - 5);  // Display seconds with 2 decimals
                ctx.beginPath();
                ctx.moveTo(x, canvas.height - 20);
                ctx.lineTo(x, canvas.height - 15); // Tick mark
                ctx.stroke();
            }
        }

        // Function to draw the graph
        function drawGraph() {
            ctx.clearRect(0, 0, canvas.width, canvas.height);

            const timeWindow = parseFloat(timeWindowInput.value);  // Get time window from user input
            const currentTime = timeData.length > 0 ? timeData[timeData.length - 1] : Date.now() / 1000;
            const minTime = currentTime - timeWindow;  // Calculate the minimum time based on the window size

            // Filter data based on the time window and get min/max for Y-axis
            const visibleIndices = timeData.map((t, i) => t >= minTime ? i : null).filter(i => i !== null);
            
            if (visibleIndices.length === 0) return;  // No data in range

            const visibleMotorPulses = visibleIndices.map(i => motorPulsesData[i]);
            const visibleEncoderPosition = visibleIndices.map(i => encoderPositionData[i]);
            const visibleSetPoint = visibleIndices.map(i => setPointData[i]);

            let minY = Infinity;
            let maxY = -Infinity;

            if (motorPulsesCheckbox.checked && visibleMotorPulses.length) {
                minY = Math.min(minY, ...visibleMotorPulses);
                maxY = Math.max(maxY, ...visibleMotorPulses);
            }
            if (encoderPositionCheckbox.checked && visibleEncoderPosition.length) {
                minY = Math.min(minY, ...visibleEncoderPosition);
                maxY = Math.max(maxY, ...visibleEncoderPosition);
            }
            if (setPointCheckbox.checked && visibleSetPoint.length) {
                minY = Math.min(minY, ...visibleSetPoint);
                maxY = Math.max(maxY, ...visibleSetPoint);
            }

            // Set default range if no data is visible
            if (minY === Infinity || maxY === -Infinity) {
                minY = 0;
                maxY = 200; // Default Y range
            }

            drawAxes(currentTime, minTime, minY, maxY);  // Draw the axes with dynamic Y range
            drawLegend();  // Draw the legend

            const plotData = (data, color) => {
                if (data.length === 0) return;
                ctx.strokeStyle = color;
                ctx.beginPath();
                for (let i = 0; i < data.length; i++) {
                    if (visibleIndices[i] === undefined) continue;  // Skip data not in range
                    const x = 50 + (timeData[visibleIndices[i]] - minTime) / (currentTime - minTime) * (canvas.width - 60);
                    const y = canvas.height - 20 - ((data[i] - minY) / (maxY - minY)) * (canvas.height - 20);  // Adjust for axes
                    if (i === 0) {
                        ctx.moveTo(x, y);
                    } else {
                        ctx.lineTo(x, y);
                    }
                }
                ctx.stroke();
            };

            if (motorPulsesCheckbox.checked) plotData(visibleMotorPulses, 'red');
            if (encoderPositionCheckbox.checked) plotData(visibleEncoderPosition, 'green');
            if (setPointCheckbox.checked) plotData(visibleSetPoint, 'blue');
        }

        // Function to fetch data and update arrays
        function fetchData() {
            fetchJSONData().then(data => {
                // Extract the data from the JSON response
                timeData.push(data.timeStamp);
                motorPulsesData.push(data.motorPulses);
                encoderPositionData.push(data.encoderPosition);
                setPointData.push(data.setPoint);

                // Update the latest values display
                latestMotorPulses.textContent = `Motor Pulses: ${data.motorPulses}`;
                latestEncoderPosition.textContent = `Encoder Position: ${data.encoderPosition}`;
                latestSetPoint.textContent = `SetPoint: ${data.setPoint}`;

                // Draw the graph with updated data
                drawGraph();
            });
        }

        // Start fetching data at intervals
        function startFetching() {
            clearInterval(intervalId);
            const newRate = parseInt(refreshRateInput.value);
            if (newRate >= 20) {  // Setting a minimum limit of 20ms for refresh rate
                intervalId = setInterval(() => {
                    if (!freezeDisplayCheckbox.checked) {
                        fetchData();
                    }
                }, newRate);
            }
        }
        startFetching();

        // Update interval when refresh rate changes
        refreshRateInput.addEventListener('change', () => {
            startFetching();
        });

        // Handle freeze display checkbox
        freezeDisplayCheckbox.addEventListener('change', () => {
            if (freezeDisplayCheckbox.checked) {
                clearInterval(intervalId);  // Stop fetching data
            } else {
                // Clear current data and restart fetching
                timeData = [];
                motorPulsesData = [];
                encoderPositionData = [];
                setPointData = [];
                startFetching();
            }
        });

        // Redraw the graph when the time window changes
        timeWindowInput.addEventListener('change', drawGraph);
    </script>
</body>
</html>
)html"