R"html(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Values over time</title>
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
    <h1>Values over time</h1>

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

        let data = [];
        let animationId;
        let lastFetchTime = 0;
        let needsFullRedraw = true;

        // Offscreen canvas for background
        const offscreenCanvas = new OffscreenCanvas(800, 400);
        const offscreenCtx = offscreenCanvas.getContext('2d');

        // Web Worker for data fetching and processing
        const worker = new Worker(URL.createObjectURL(new Blob([`
            let data = [];
            let timeWindow = 30;

            function fetchJSONData() {
                return fetch('http://{{IP_ADDRESS}}/feedback', {
                    method: 'GET',
                    headers: {'Content-Type': 'application/json'},
                    mode: 'cors'
                })
                .then(response => {
                    if (!response.ok) {
                        throw new Error(\`HTTP error! status: \${response.status}\`);
                    }
                    return response.json();
                })
                .then(rawData => {
                    if (rawData.pulses !== undefined && rawData.encoder !== undefined && rawData.timeStamp !== undefined && rawData.setPoint !== undefined) {
                        return {
                            motorPulses: rawData.pulses,
                            encoderPosition: rawData.encoder,
                            timeStamp: rawData.timeStamp,
                            setPoint: rawData.setPoint
                        };
                    } else {
                        throw new Error('Invalid data structure received');
                    }
                })
                .catch(error => {
                    console.error('Error fetching data:', error);
                    return {
                        motorPulses: 0,
                        encoderPosition: 0,
                        timeStamp: Date.now() / 1000,
                        setPoint: 0
                    };
                });
            }

            function processData() {
                fetchJSONData().then(newData => {
                    data.push(newData);
                    const currentTime = newData.timeStamp;
                    const minTime = currentTime - timeWindow;
                    data = data.filter(d => d.timeStamp >= minTime);

                    self.postMessage({type: 'newData', data: newData, fullData: data});
                });
            }

            setInterval(processData, 100);

            self.onmessage = function(e) {
                if (e.data.type === 'updateTimeWindow') {
                    timeWindow = e.data.timeWindow;
                }
            };
        `], {type: 'text/javascript'})));

        worker.onmessage = function(e) {
            if (e.data.type === 'newData' && !freezeDisplayCheckbox.checked) {
                updateLatestValues(e.data.data);
                data = e.data.fullData;
                needsFullRedraw = true;
            }
        };

        function updateLatestValues(latestData) {
            latestMotorPulses.textContent = `Motor Pulses: ${latestData.motorPulses}`;
            latestEncoderPosition.textContent = `Encoder Position: ${latestData.encoderPosition}`;
            latestSetPoint.textContent = `SetPoint: ${latestData.setPoint}`;
        }

        function drawBackground(ctx, maxTime, minTime, minY, maxY) {
            ctx.fillStyle = '#f0f0f0';
            ctx.fillRect(0, 0, canvas.width, canvas.height);

            ctx.strokeStyle = 'black';
            ctx.beginPath();
            ctx.moveTo(50, 0);
            ctx.lineTo(50, canvas.height - 20);
            ctx.lineTo(canvas.width, canvas.height - 20);
            ctx.stroke();

            ctx.font = '14px Arial';
            ctx.fillStyle = 'black';
            ctx.save();
            ctx.translate(20, canvas.height / 2);
            ctx.rotate(-Math.PI / 2);
            ctx.fillText('Motor Steps', 0, 0);
            ctx.restore();

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
                ctx.lineTo(55, y);
                ctx.stroke();
            }

            const xTicks = 5;
            const timeRange = maxTime - minTime;
            const xStep = (canvas.width - 60) / xTicks;
            for (let i = 0; i <= xTicks; i++) {
                const x = 50 + i * xStep;
                const timeLabel = minTime + i * (timeRange / xTicks);
                ctx.fillText(timeLabel.toFixed(2) + 's', x - 20, canvas.height - 5);
                ctx.beginPath();
                ctx.moveTo(x, canvas.height - 20);
                ctx.lineTo(x, canvas.height - 15);
                ctx.stroke();
            }

            ctx.font = '12px Arial';
            ctx.fillStyle = 'red';
            ctx.fillText('Motor Pulses', 680, 30);
            ctx.fillStyle = 'green';
            ctx.fillText('Encoder Position', 680, 50);
            ctx.fillStyle = 'blue';
            ctx.fillText('SetPoint', 680, 70);
        }

        function downsample(data, maxPoints) {
            if (data.length <= maxPoints) return data;
            const every = Math.floor(data.length / maxPoints);
            return data.filter((_, i) => i % every === 0);
        }

        function drawGraph() {
            if (data.length === 0) return;

            const timeWindow = parseFloat(timeWindowInput.value);
            const currentTime = data[data.length - 1].timeStamp;
            const minTime = currentTime - timeWindow;

            let minY = Infinity;
            let maxY = -Infinity;

            data.forEach(d => {
                if (motorPulsesCheckbox.checked) {
                    minY = Math.min(minY, d.motorPulses);
                    maxY = Math.max(maxY, d.motorPulses);
                }
                if (encoderPositionCheckbox.checked) {
                    minY = Math.min(minY, d.encoderPosition);
                    maxY = Math.max(maxY, d.encoderPosition);
                }
                if (setPointCheckbox.checked) {
                    minY = Math.min(minY, d.setPoint);
                    maxY = Math.max(maxY, d.setPoint);
                }
            });

            if (minY === Infinity || maxY === -Infinity) {
                minY = 0;
                maxY = 200;
            }

            if (needsFullRedraw) {
                drawBackground(offscreenCtx, currentTime, minTime, minY, maxY);
                needsFullRedraw = false;
            }

            ctx.clearRect(0, 0, canvas.width, canvas.height);
            ctx.drawImage(offscreenCanvas, 0, 0);

            const downsampledData = downsample(data, 200);

            const plotData = (getData, color) => {
                ctx.strokeStyle = color;
                ctx.beginPath();
                downsampledData.forEach((d, i) => {
                    const x = 50 + (d.timeStamp - minTime) / timeWindow * (canvas.width - 60);
                    const y = canvas.height - 20 - ((getData(d) - minY) / (maxY - minY)) * (canvas.height - 20);
                    if (i === 0) {
                        ctx.moveTo(x, y);
                    } else {
                        ctx.lineTo(x, y);
                    }
                });
                ctx.stroke();
            };

            if (motorPulsesCheckbox.checked) plotData(d => d.motorPulses, 'red');
            if (encoderPositionCheckbox.checked) plotData(d => d.encoderPosition, 'green');
            if (setPointCheckbox.checked) plotData(d => d.setPoint, 'blue');
        }

        function animate(timestamp) {
            if (timestamp - lastFetchTime >= parseInt(refreshRateInput.value)) {
                drawGraph();
                lastFetchTime = timestamp;
            }
            animationId = requestAnimationFrame(animate);
        }

        function startAnimation() {
            if (animationId) cancelAnimationFrame(animationId);
            animationId = requestAnimationFrame(animate);
        }

        startAnimation();

        refreshRateInput.addEventListener('change', () => {
            if (!freezeDisplayCheckbox.checked) {
                startAnimation();
            }
        });

        freezeDisplayCheckbox.addEventListener('change', () => {
            if (freezeDisplayCheckbox.checked) {
                cancelAnimationFrame(animationId);
            } else {
                startAnimation();
            }
        });

        timeWindowInput.addEventListener('change', () => {
            worker.postMessage({type: 'updateTimeWindow', timeWindow: parseFloat(timeWindowInput.value)});
            needsFullRedraw = true;
        });

        [motorPulsesCheckbox, encoderPositionCheckbox, setPointCheckbox].forEach(checkbox => {
            checkbox.addEventListener('change', () => {
                needsFullRedraw = true;
            });
        });
    </script>
</body>
</html>
)html"