R"html(
<html>
  <head>
    <title>Encoder board configuration</title>
    <style>
      body {
        font-family: 'Roboto', Arial, sans-serif;
        background-color: #0a0a0a;
        color: #e0e0e0;
        margin: 0;
        padding: 0;
        display: flex;
        justify-content: center;
        align-items: center;
        min-height: 100vh;
        background-image: linear-gradient(45deg, #0a0a0a 25%, #0f0f0f 25%, #0f0f0f 50%, #0a0a0a 50%, #0a0a0a 75%, #0f0f0f 75%, #0f0f0f 100%);
        background-size: 40px 40px;
      }

      .container {
        display: grid;
        grid-template-columns: repeat(3, 1fr);
        /* Creates 3 equal-width columns */
        gap: 20px;
        /* Adjusts spacing between grid items */
        padding: 20px;
      }

      .form-container {
        background-color: #1a1a1a;
        border-radius: 12px;
        box-shadow: 0 4px 15px rgba(0, 0, 0, 0.3);
        padding: 20px;
        width: 100%;
        box-sizing: border-box;
        transition: transform 0.3s ease, box-shadow 0.3s ease;
      }

      .form-container:hover {
        transform: translateY(-5px);
        box-shadow: 0 6px 20px rgba(0, 0, 0, 0.4);
      }

      .form-group {
        display: flex;
        align-items: center;
      }

      .form-group label {
        margin-right: 10px;
        /* Adjust the spacing between label and input */
      }

      label {
        font-weight: bold;
        color: #b0b0b0;
        margin-bottom: 8px;
      }

      input[type="text"],
      input[type="submit"],
      input[type="password"],
      select {
        padding: 8px;
        margin-bottom: 12px;
        border: 1px solid #444;
        border-radius: 4px;
        font-size: 14px;
        width: 100%;
        background-color: #252525;
        color: #e0e0e0;
        transition: border-color 0.3s ease, box-shadow 0.3s ease;
      }

      .ip-input-container {
        display: flex;
        align-items: center;
      }

      .ip-input-container label {
        margin-right: 4px;
      }

      input[type="submit"] {
        background-color: #3a3a3a;
        color: #e0e0e0;
        border: none;
        cursor: pointer;
        transition: background-color 0.2s ease-in-out;
      }

      input[type="submit"]:hover {
        background-color: #0056b3;
      }

      iframe {
        display: none;
      }

      iframe {
        display: none;
      }
    </style>
  </head>
  <body>
    <div class='container'>
      <div class='form-container'>
        <h2>NetworkConfiguration (reboot)</h2>
        <form method='put' target='hiddenFrame' onsubmit='putAndGetNetworkConfig()'>
          <div class='ip-input-container'>
            <label>IPAddress:</label>
            <input type='text' id='ipByte1' size='4' placeholder='...'>
            <label>.</label>
            <input type='text' id='ipByte2' size='4' placeholder='...'>
            <label>.</label>
            <input type='text' id='ipByte3' size='4' placeholder='...'>
            <label>.</label>
            <input type='text' id='ipByte4' size='4' placeholder='...'>
          </div>
          <div class='ip-input-container'>
            <label>GatewayIP:</label>
            <input type='text' id='gatewayByte1' size='4' placeholder='...'>
            <label>.</label>
            <input type='text' id='gatewayByte2' size='4' placeholder='...'>
            <label>.</label>
            <input type='text' id='gatewayByte3' size='4' placeholder='...'>
            <label>.</label>
            <input type='text' id='gatewayByte4' size='4' placeholder='...'>
          </div>
          <div class='ip-input-container'>
            <label>SubnetMasks:</label>
            <input type='text' id='masksByte1' size='4' placeholder='...'>
            <label>.</label>
            <input type='text' id='masksByte2' size='4' placeholder='...'>
            <label>.</label>
            <input type='text' id='masksByte3' size='4' placeholder='...'>
            <label>.</label>
            <input type='text' id='masksByte4' size='4' placeholder='...'>
          </div>
          <input type='submit' value='Update'>
        </form>
      </div>
      <div class='form-container'>
        <h2>Control Parameters</h2>
        <form method='put' target='hiddenFrame' onsubmit='putAndGetControlParams()'>
          <div class='form-group'>
            <label>Kp:</label>
            <input type='text' id='Kp' placeholder='...'>
          </div>
          <div class='form-group'>
            <label>Ki:</label>
            <input type='text' id='Ki' placeholder='...'>
          </div>
          <div class='form-group'>
            <label>Kd:</label>
            <input type='text' id='Kd' placeholder='...'>
          </div>
          <div class='form-group'>
            <label>maxVelocity:</label>
            <input type='text' id='maxVelocity' placeholder='...'>
          </div>
          <div class='form-group'>
            <label>errorLimit:</label>
            <input type='text' id='errorLimit' placeholder='...'>
          </div>
          <input type='submit' value='Update'>
        </form>
      </div>
      <div class='form-container'>
        <h2>MQTT configuration (reboot)</h2>
        <form method='put' target='hiddenFrame' onsubmit='putAndGetMqttConfig()'>
          <div class='ip-input-container'>
            <label>BrokerIP:</label>
            <input type='text' id='brokerIp1' size='4' placeholder='...'>
            <label>.</label>
            <input type='text' id='brokerIp2' size='4' placeholder='...'>
            <label>.</label>
            <input type='text' id='brokerIp3' size='4' placeholder='...'>
            <label>.</label>
            <input type='text' id='brokerIp4' size='4' placeholder='...'>
          </div>
          <div class='form-group'>
            <label>BrokerPort:</label>
            <input type='text' id='brokerPort' placeholder='...'>
          </div>
          <div>
            <label>MQTT:</label>
            <input type='radio' id='mqttEnabled' value='Enabled' name='mqttEnabledRadio'>
            <label for='mqttEnabled'>Enabled</label>
            <input type='radio' id='mqttDisabled' value='Disabled' name='mqttEnabledRadio'>
            <label for='mqttDisabled'>Disabled</label>
          </div>
          <div class='form-group'>
            <label>
              <br />BoardNumber: </label>
            <input type='text' id='boardNumber' placeholder='...'>
          </div>
          <div>
            <label>PulseFeedback:</label>
            <input type='radio' id='mqttPulseFbEnabled' value='Enabled' name='mqttPulseFbRadio'>
            <label for='mqttPulseFbEnabled'>Enabled</label>
            <input type='radio' id='mqttPulseFbDisabled' value='Disabled' name='mqttPulseFbRadio'>
            <label for='mqttPulseFbDisabled'>Disabled</label>
          </div>
          <div>
            <label>EncoderFeedback:</label>
            <input type='radio' id='mqttEncoderFbEnabled' value='Enabled' name='mqttEncoderFbRadio'>
            <label for='mqttEncoderFbEnabled'>Enabled</label>
            <input type='radio' id='mqttEncoderFbDisabled' value='Disabled' name='mqttEncoderFbRadio'>
            <label for='mqttEncoderFbDisabled'>Disabled</label>
          </div>
          <div>
            <label>SetpointFeedback:</label>
            <input type='radio' id='mqttSetpointFbEnabled' value='Enabled' name='mqttSetpointFbRadio'>
            <label for='mqttSetpointFbEnabled'>Enabled</label>
            <input type='radio' id='mqttSetpointFbDisabled' value='Disabled' name='mqttSetpointFbRadio'>
            <label for='mqttSetpointFbDisabled'>Disabled</label>
          </div>
          <div>
            <label>TimestampFeedback:</label>
            <input type='radio' id='mqttTimestampFbEnabled' value='Enabled' name='mqttTimestampFbRadio'>
            <label for='mqttTimestampFbEnabled'>Enabled</label>
            <input type='radio' id='mqttTimestampFbDisabled' value='Disabled' name='mqttTimestampFbRadio'>
            <label for='mqttTimestampFbDisabled'>Disabled</label>
          </div>
          <div>
            <label>ErrorFeedback:</label>
            <input type='radio' id='mqttErrorFbEnabled' value='Enabled' name='mqttErrorFbRadio'>
            <label for='mqttErrorFbEnabled'>Enabled</label>
            <input type='radio' id='mqttErrorFbDisabled' value='Disabled' name='mqttErrorFbRadio'>
            <label for='mqttErrorFbDisabled'>Disabled</label>
          </div>
          <div class='form-group'>
            <label>
              <br />PublishPeriod(s): </label>
            <input type='text' id='publishPeriod' placeholder='...'>
          </div>
          <div class='form-group'>
            <label>Username:</label>
            <input type='text' id='mqttUsername' placeholder='...'>
          </div>
          <div class='form-group'>
            <label>Password:</label>
            <input type='password' id='mqttPassword'>
          </div>
          <input type='submit' value='Update'>
        </form>
      </div>
      <div class='form-container'>
        <h2>Hardware Configuration</h2>
        <form method='put' target='hiddenFrame' onsubmit='putAndGetHwConfig()'>
          <div class='form-group'>
            <label>motorStepsPerRev:</label>
            <input type='text' id='motorStepsPerRev' placeholder='...'>
          </div>
          <div class='form-group'>
            <label>encoderTicksPerRev:</label>
            <input type='text' id='encoderTicksPerRev' placeholder='...'>
          </div>
          <input type='submit' value='Update'>
        </form>
      </div>
      <div class="form-container">
        <h2 id="motorPower">Motor is ...</h2>
        <form method="put" target="hiddenFrame" onsubmit="motorOn()" style="display: inline;">
          <div class="form-group">
            <input type="submit" value="Turn On">
          </div>
        </form>
        <form method="put" target="hiddenFrame" onsubmit="motorOff()" style="display: inline;">
          <div class="form-group">
            <input type="submit" value="Turn Off">
          </div>
        </form>
      </div>
      <div class="form-container">
        <h2 id="closedLoop">Closed loop : ...</h2>
        <form method="put" target="hiddenFrame" onsubmit="enableClosedLoop()" style="display: inline;">
          <div class="form-group">
            <input type="submit" value="Enable closed loop">
          </div>
        </form>
        <form method="put" target="hiddenFrame" onsubmit="disableClosedLoop()" style="display: inline;">
          <div class="form-group">
            <input type="submit" value="Disable closed loop">
          </div>
        </form>
      </div>
      <div class='form-container'>
        <h2>Basic Position</h2>
        <form method='post' target='hiddenFrame' onsubmit='setPoint()'>
          <div class='form-group'>
            <label>Target</label>
            <input type='text' id='setPointTarget'>
            <input type='submit' value='Go'>
          </div>
          <div>
            <input type='radio' id='Relative' value='Relative' name='setPointRelativity' checked>
            <label for='Relative'>Relative</label>
            <input type='radio' id='Absolute' value='Absolute' name='setPointRelativity'>
            <label for='Absolute'>Absolute</label>
          </div>
        </form>
      </div>
      <div class='form-container'>
        <h2>Basic Velocity</h2>
        <form method='post' target='hiddenFrame' onsubmit='jog()'>
          <div class='form-group'>
            <label>Velocity</label>
            <input type='text' id='setVelocity'>
            <input type='submit' value='Go'>
          </div>
        </form>
        <form method="put" target="hiddenFrame" onsubmit="jogZero()" style="display: inline;">
          <div class="form-group">
            <input type="submit" value="Stop">
          </div>
        </form>
      </div>
      <div class='form-container'>
        <form method='put' target='hiddenFrame' onsubmit='reboot()'>
          <div class='form-group'>
            <input type='submit' value='Reboot'>
          </div>
        </form>
      </div>
      <iframe name='hiddenFrame' style=' display:none;'></iframe>
      <script>
        function putNetworkConfig() {
          var ipByte1 = document.getElementById("ipByte1").value;
          var ipByte2 = document.getElementById("ipByte2").value;
          var ipByte3 = document.getElementById("ipByte3").value;
          var ipByte4 = document.getElementById("ipByte4").value;
          var gatewayByte1 = document.getElementById("gatewayByte1").value;
          var gatewayByte2 = document.getElementById("gatewayByte2").value;
          var gatewayByte3 = document.getElementById("gatewayByte3").value;
          var gatewayByte4 = document.getElementById("gatewayByte4").value;
          var masksByte1 = document.getElementById("masksByte1").value;
          var masksByte2 = document.getElementById("masksByte2").value;
          var masksByte3 = document.getElementById("masksByte3").value;
          var masksByte4 = document.getElementById("masksByte4").value;
          var plainJSON = "{";
          if (ipByte1 !== "" && ipByte2 !== "" && ipByte3 !== "" && ipByte4 !== "") {
            plainJSON += '"ipAddress":"' + ipByte1 + '.' + ipByte2 + '.' + ipByte3 + '.' + ipByte4 + '",';
          }
          if (gatewayByte1 !== "" && gatewayByte2 !== "" && gatewayByte3 !== "" && gatewayByte4 !== "") {
            plainJSON += '"gatewayIp":"' + gatewayByte1 + '.' + gatewayByte2 + '.' + gatewayByte3 + '.' + gatewayByte4 + '",';
          }
          if (masksByte1 !== "" && masksByte2 !== "" && masksByte3 !== "" && masksByte4 !== "") {
            plainJSON += '"networkMasks":"' + masksByte1 + '.' + masksByte2 + '.' + masksByte3 + '.' + masksByte4 + '",';
          }
          plainJSON = plainJSON.slice(0, -1);
          plainJSON += '}';
          fetch("http://{{IP_ADDRESS}}/networkConfig", {
            method: "PUT",
            body: plainJSON
          });
        }
        async function getNetworkConfig() {
          try {
            const response = await fetch("http://{{IP_ADDRESS}}/networkConfig");
            if (!response.ok) {
              throw new Error(`HTTP error! status: ${response.status}`);
            }
            const data = await response.json();
            const ipSplit = data.ipAddress.split('.');
            document.getElementById("ipByte1").placeholder = ipSplit[0];
            document.getElementById("ipByte2").placeholder = ipSplit[1];
            document.getElementById("ipByte3").placeholder = ipSplit[2];
            document.getElementById("ipByte4").placeholder = ipSplit[3];
            document.getElementById("ipByte1").value = '';
            document.getElementById("ipByte2").value = '';
            document.getElementById("ipByte3").value = '';
            document.getElementById("ipByte4").value = '';
            const gatewaySplit = data.gatewayIp.split('.');
            document.getElementById("gatewayByte1").placeholder = gatewaySplit[0];
            document.getElementById("gatewayByte2").placeholder = gatewaySplit[1];
            document.getElementById("gatewayByte3").placeholder = gatewaySplit[2];
            document.getElementById("gatewayByte4").placeholder = gatewaySplit[3];
            document.getElementById("gatewayByte1").value = '';
            document.getElementById("gatewayByte2").value = '';
            document.getElementById("gatewayByte3").value = '';
            document.getElementById("gatewayByte4").value = '';
            const maskSplit = data.networkMasks.split('.');
            document.getElementById("masksByte1").placeholder = maskSplit[0];
            document.getElementById("masksByte2").placeholder = maskSplit[1];
            document.getElementById("masksByte3").placeholder = maskSplit[2];
            document.getElementById("masksByte4").placeholder = maskSplit[3];
            document.getElementById("masksByte1").value = '';
            document.getElementById("masksByte2").value = '';
            document.getElementById("masksByte3").value = '';
            document.getElementById("masksByte4").value = '';
          } catch (error) {
            console.error('Error:', error);
          }
        }

        function putAndGetNetworkConfig() {
          putNetworkConfig();
          getNetworkConfig();
        }

        function putControlParams() {
          var Kp = document.getElementById("Kp").value;
          var Ki = document.getElementById("Ki").value;
          var Kd = document.getElementById("Kd").value;
          var maxVelocity = document.getElementById("maxVelocity").value;
          var errorLimit = document.getElementById("errorLimit").value;
          var plainJSON = "{";
          if (Kp !== "") {
            plainJSON += '"Kp":' + Kp + ',';
          }
          if (Ki !== "") {
            plainJSON += '"Ki":' + Ki + ',';
          }
          if (Kd !== "") {
            plainJSON += '"Kd":' + Kd + ',';
          }
          if (maxVelocity !== "") {
            plainJSON += '"maxVelocity":' + maxVelocity + ',';
          }
          if (errorLimit !== "") {
            plainJSON += '"errorLimit":' + errorLimit + ',';
          }
          plainJSON = plainJSON.slice(0, -1);
          plainJSON += '}';
          fetch("http://{{IP_ADDRESS}}/controlParameters", {
            method: "PUT",
            body: plainJSON
          });
        }
        async function getControlParams() {
          try {
            const response = await fetch("http://{{IP_ADDRESS}}/controlParameters");
            if (!response.ok) {
              throw new Error(`HTTP error! status: ${response.status}`);
            }
            const data = await response.json();
            document.getElementById("Kp").placeholder = data.Kp;
            document.getElementById("Kp").value = '';
            document.getElementById("Ki").placeholder = data.Ki;
            document.getElementById("Ki").value = '';
            document.getElementById("Kd").placeholder = data.Kd;
            document.getElementById("Kd").value = '';
            document.getElementById("maxVelocity").placeholder = data.maxVelocity;
            document.getElementById("maxVelocity").value = '';
            document.getElementById("errorLimit").placeholder = data.errorLimit;
            document.getElementById("errorLimit").value = '';
          } catch (error) {
            console.error('Error:', error);
          }
        }

        function putAndGetControlParams() {
          putControlParams();
          getControlParams();
        }

        function putHwConfig() {
          var motorStepsPerRev = document.getElementById("motorStepsPerRev").value;
          var encoderTicksPerRev = document.getElementById("encoderTicksPerRev").value;
          var plainJSON = "{";
          if (motorStepsPerRev !== "") {
            plainJSON += '"motorStepsPerRev":' + motorStepsPerRev + ',';
          }
          if (encoderTicksPerRev !== "") {
            plainJSON += '"encoderTicksPerRev":' + encoderTicksPerRev + ',';
          }
          plainJSON = plainJSON.slice(0, -1);
          plainJSON += '}';
          fetch("http://{{IP_ADDRESS}}/hardwareConfig", {
            method: "PUT",
            body: plainJSON
          });
        }
        async function getHwConfig() {
          try {
            const response = await fetch("http://{{IP_ADDRESS}}/hardwareConfig");
            if (!response.ok) {
              throw new Error(`HTTP error! status: ${response.status}`);
            }
            const data = await response.json();
            document.getElementById("motorStepsPerRev").placeholder = data.motorStepsPerRev;
            document.getElementById("motorStepsPerRev").value = '';
            document.getElementById("encoderTicksPerRev").placeholder = data.encoderTicksPerRev;
            document.getElementById("encoderTicksPerRev").value = '';
          } catch (error) {
            console.error('Error:', error);
          }
        }

        function putAndGetHwConfig() {
          putHwConfig();
          getHwConfig();
        }

        function motorOn() {
          fetch("http://{{IP_ADDRESS}}/motor/on", {
            method: "POST",
            body: ""
          });
          updateMotorStatus()
        }

        function motorOff() {
          fetch("http://{{IP_ADDRESS}}/motor/off", {
            method: "POST",
            body: ""
          });
          updateMotorStatus();
        }

        function enableClosedLoop() {
          fetch("http://{{IP_ADDRESS}}/motor/closedLoop/enable", {
            method: "POST",
            body: ""
          });
          updateMotorStatus();
        }

        function disableClosedLoop() {
          fetch("http://{{IP_ADDRESS}}/motor/closedLoop/disable", {
            method: "POST",
            body: ""
          });
          updateMotorStatus();
        }
        async function updateMotorStatus() {
          try {
            const response = await fetch("http://{{IP_ADDRESS}}/motor");
            if (!response.ok) {
              throw new Error(`HTTP error! status: ${response.status}`);
            }
            const data = await response.json();
            if (data.motorOn) {
              document.getElementById("motorPower").textContent = "Motor is ON";
            } else {
              document.getElementById("motorPower").textContent = "Motor is OFF";
            }
            if (data.closedLoopEnabled) {
              document.getElementById("closedLoop").textContent = "Closed loop : Enabled";
            } else {
              document.getElementById("closedLoop").textContent = "Closed loop :  Disabled";
            }
          } catch (error) {
            console.error('Error:', error);
          }
        }

        function setPoint() {
          var target = document.getElementById("setPointTarget").value;
          const selectedOption = document.querySelector('input[name="setPointRelativity"]:checked');
          const id = selectedOption.id;
          var plainJSON = '{';
          plainJSON += '"targetPosition":' + target + ',';
          if (id == 'Relative') {
            plainJSON += '"type": "RELATIVE"}';
          } else {
            plainJSON += '"type": "ABSOLUTE"}';
          }
          fetch("http://{{IP_ADDRESS}}/setPoint", {
            method: "POST",
            body: plainJSON
          });
        }

        function jog() {
          var velocity = document.getElementById("setVelocity").value;
          var plainJSON = '{';
          plainJSON += '"velocity":' + velocity + '}';
          fetch("http://{{IP_ADDRESS}}/jog", {
            method: "POST",
            body: plainJSON
          });
        }

        function jogZero() {
          const velocity = 0;
          var plainJSON = '{';
          plainJSON += '"velocity":' + velocity + '}';
          fetch("http://{{IP_ADDRESS}}/jog", {
            method: "POST",
            body: plainJSON
          });
        }

        function putMqttConfig() {
          var brokerByte1 = document.getElementById("brokerIp1").value;
          var brokerByte2 = document.getElementById("brokerIp2").value;
          var brokerByte3 = document.getElementById("brokerIp3").value;
          var brokerByte4 = document.getElementById("brokerIp4").value;
          var brokerPort = document.getElementById("brokerPort").value;
          const selectedMqttEnabled = document.querySelector('input[name="mqttEnabledRadio"]:checked');
          const selectedMqttEnabledId = selectedMqttEnabled.id;
          var boardNumber = document.getElementById("boardNumber").value;
          const selectedPulseFeedbackEnabled = document.querySelector('input[name="mqttPulseFbRadio"]:checked');
          const selectedPulseFeedbackEnabledId = selectedPulseFeedbackEnabled.id;
          const selectedEncoderFeedbackEnabled = document.querySelector('input[name="mqttEncoderFbRadio"]:checked');
          const selectedEncoderFeedbackEnabledId = selectedEncoderFeedbackEnabled.id;
          const selectedSetpointFeedbackEnabled = document.querySelector('input[name="mqttSetpointFbRadio"]:checked');
          const selectedSetpointFeedbackEnabledId = selectedSetpointFeedbackEnabled.id;
          const selectedTimestampFeedbackEnabled = document.querySelector('input[name="mqttTimestampFbRadio"]:checked');
          const selectedTimestampFeedbackEnabledId = selectedTimestampFeedbackEnabled.id;
          const selectedErrorFeedbackEnabled = document.querySelector('input[name="mqttErrorFbRadio"]:checked');
          const selectedErrorFeedbackEnabledId = selectedErrorFeedbackEnabled.id;
          var publishPeriod = document.getElementById("publishPeriod").value;
          var username = document.getElementById("mqttUsername").value;
          var password = document.getElementById("mqttPassword").value;
          var plainJSON = '{';
          if (brokerByte1 !== "" && brokerByte2 !== "" && brokerByte3 !== "" && brokerByte4 !== "") {
            plainJSON += '"brokerIp":"' + brokerByte1 + '.' + brokerByte2 + '.' + brokerByte3 + '.' + brokerByte4 + '",';
          }
          if (brokerPort !== "") {
            plainJSON += '"brokerPort":' + brokerPort + ',';
          }
          if (selectedMqttEnabledId == 'mqttEnabled') {
            plainJSON += '"mqttEnabled":true,';
          } else {
            plainJSON += '"mqttEnabled":false,';
          }
          if (boardNumber !== "") {
            plainJSON += '"boardNumber":' + boardNumber + ',';
          }
          if (selectedPulseFeedbackEnabledId == 'mqttPulseFbEnabled') {
            plainJSON += '"pulseFeedbackEnabled":true,';
          } else {
            plainJSON += '"pulseFeedbackEnabled":false,';
          }
          if (selectedEncoderFeedbackEnabledId == 'mqttEncoderFbEnabled') {
            plainJSON += '"encoderFeedbackEnabled":true,';
          } else {
            plainJSON += '"encoderFeedbackEnabled":false,';
          }
          if (selectedSetpointFeedbackEnabledId == 'mqttSetpointFbEnabled') {
            plainJSON += '"setPointFeedbackEnabled":true,';
          } else {
            plainJSON += '"setPointFeedbackEnabled":false,';
          }
          if (selectedTimestampFeedbackEnabledId == 'mqttTimestampFbEnabled') {
            plainJSON += '"timeStampFeedbackEnabled":true,';
          } else {
            plainJSON += '"timeStampFeedbackEnabled":false,';
          }
          if (selectedErrorFeedbackEnabledId == 'mqttErrorFbEnabled') {
            plainJSON += '"errorFeedbackEnabled":true,';
          } else {
            plainJSON += '"errorFeedbackEnabled":false,';
          }
          if (publishPeriod !== "") {
            plainJSON += '"publishPeriod":' + publishPeriod + ',';
          }
          if (username !== "") {
            plainJSON += '"username":"' + username + '",';
          }
          if (password !== "") {
            plainJSON += '"password":"' + password + '",';
          }
          plainJSON = plainJSON.slice(0, -1);
          plainJSON += '}';
          fetch("http://{{IP_ADDRESS}}/mqttConfig", {
            method: "PUT",
            body: plainJSON
          });
        }
        async function getMqttConfig() {
          try {
            const response = await fetch("http://{{IP_ADDRESS}}/mqttConfig");
            if (!response.ok) {
              throw new Error(`HTTP error! status: ${response.status}`);
            }
            const data = await response.json();
            const ipSplit = data.brokerIp.split('.');
            document.getElementById("brokerIp1").placeholder = ipSplit[0];
            document.getElementById("brokerIp2").placeholder = ipSplit[1];
            document.getElementById("brokerIp3").placeholder = ipSplit[2];
            document.getElementById("brokerIp4").placeholder = ipSplit[3];
            document.getElementById("brokerIp1").value = '';
            document.getElementById("brokerIp2").value = '';
            document.getElementById("brokerIp3").value = '';
            document.getElementById("brokerIp4").value = '';
            document.getElementById("brokerPort").placeholder = data.brokerPort;
            document.getElementById("brokerPort").value = '';
            if (data.mqttEnabled) {
              document.getElementById("mqttEnabled").checked = true;
              document.getElementById("mqttDisabled").checked = false;
            } else {
              document.getElementById("mqttEnabled").checked = false;
              document.getElementById("mqttDisabled").checked = true;
            }
            document.getElementById("boardNumber").placeholder = data.boardNumber;
            document.getElementById("boardNumber").value = '';
            if (data.pulseFeedbackEnabled) {
              document.getElementById("mqttPulseFbEnabled").checked = true;
              document.getElementById("mqttPulseFbDisabled").checked = false;
            } else {
              document.getElementById("mqttPulseFbEnabled").checked = false;
              document.getElementById("mqttPulseFbDisabled").checked = true;
            }
            if (data.encoderFeedbackEnabled) {
              document.getElementById("mqttEncoderFbEnabled").checked = true;
              document.getElementById("mqttEncoderFbDisabled").checked = false;
            } else {
              document.getElementById("mqttEncoderFbEnabled").checked = false;
              document.getElementById("mqttEncoderFbDisabled").checked = true;
            }
            if (data.setPointFeedbackEnabled) {
              document.getElementById("mqttSetpointFbEnabled").checked = true;
              document.getElementById("mqttSetpointFbDisabled").checked = false;
            } else {
              document.getElementById("mqttSetpointFbEnabled").checked = false;
              document.getElementById("mqttSetpointFbDisabled").checked = true;
            }
            if (data.timeStampFeedbackEnabled) {
              document.getElementById("mqttTimestampFbEnabled").checked = true;
              document.getElementById("mqttTimestampFbDisabled").checked = false;
            } else {
              document.getElementById("mqttTimestampFbEnabled").checked = false;
              document.getElementById("mqttTimestampFbDisabled").checked = true;
            }
            if (data.errorFeedbackEnabled) {
              document.getElementById("mqttErrorFbEnabled").checked = true;
              document.getElementById("mqttErrorFbDisabled").checked = false;
            } else {
              document.getElementById("mqttErrorFbEnabled").checked = false;
              document.getElementById("mqttErrorFbDisabled").checked = true;
            }
            document.getElementById("publishPeriod").placeholder = data.publishPeriod;
            document.getElementById("publishPeriod").value = '';
            document.getElementById("mqttUsername").placeholder = data.username;
            document.getElementById("mqttUsername").value = '';
            document.getElementById("mqttPassword").value = '';
          } catch (error) {
            console.error('Error:', error);
          }
        }

        function putAndGetMqttConfig() {
          putMqttConfig();
          getMqttConfig();
        }

        function reboot() {
          fetch("http://{{IP_ADDRESS}}/reboot", {
            method: "POST",
            body: ""
          });
          setTimeout(() => {
            location.reload();
          }, 2000);
        }
        // call the getters once page has loaded
        window.onload = function() {
          getNetworkConfig();
          getControlParams();
          getMqttConfig();
          getHwConfig();
          updateMotorStatus();
        };
      </script>
      </container>
  </body>
</html>
)html"