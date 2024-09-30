R"html(
    <script>
      function putNetworkConfig(){
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
	      plainJSON += '"ipAddress":"' + ipByte1 + '.' + ipByte2 + '.'+ ipByte3 + '.' + ipByte4 + '",';
	    }
	    if (gatewayByte1 !== "" && gatewayByte2 !== "" && gatewayByte3 !== "" && gatewayByte4 !== "") {
	      plainJSON += '"gatewayIp":"' + gatewayByte1 + '.' + gatewayByte2 + '.' + gatewayByte3 + '.' + gatewayByte4 + '",';
	    }
	    if (masksByte1 !== "" && masksByte2 !== "" && masksByte3 !== "" && masksByte4 !== "") {
	      plainJSON += '"networkMasks":"' + masksByte1 + '.' + masksByte2 + '.' + masksByte3 + '.' + masksByte4 + '",';
	    }
		plainJSON=plainJSON.slice(0,-1);
	    plainJSON += '}';
	    fetch("/networkConfig", {method: "PUT",body: plainJSON});
	    setTimeout(() => {location.reload();}, 100);
      }

	  function putControlParams(){
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
		plainJSON=plainJSON.slice(0,-1);
	    plainJSON += '}';
	    fetch("/controlParameters", {method: "PUT",body: plainJSON});
	    setTimeout(() => {location.reload();}, 100);
      }

	  function putHwConfig(){
	    var motorStepsPerRev = document.getElementById("motorStepsPerRev").value;
	    var encoderTicksPerRev = document.getElementById("encoderTicksPerRev").value;
	    
	    var plainJSON = "{";
	    if (motorStepsPerRev !== "") {
	      plainJSON += '"motorStepsPerRev":' + motorStepsPerRev + ',';
	    }
	    if (encoderTicksPerRev !== "") {
	      plainJSON += '"encoderTicksPerRev":' + encoderTicksPerRev + ',';
		}
		plainJSON=plainJSON.slice(0,-1);
	    plainJSON += '}';
	    fetch("/hardwareConfig", {method: "PUT",body: plainJSON});
	    setTimeout(() => {location.reload();}, 100);
      }

	  function motorOn(){
		fetch("/motor/on", {method: "POST",body: ""});
	    setTimeout(() => {location.reload();}, 100);
	  }

	  function motorOff(){
		fetch("/motor/off", {method: "POST",body: ""});
	    setTimeout(() => {location.reload();}, 100);
	  }

	  function enableClosedLoop(){
		fetch("/motor/closedLoop/enable", {method: "POST",body: ""});
	    setTimeout(() => {location.reload();}, 100);
	  }

	  function disableClosedLoop(){
		fetch("/motor/closedLoop/disable", {method: "POST",body: ""});
	    setTimeout(() => {location.reload();}, 100);
	  }

	  function setPoint(){
	  	var target = document.getElementById("setPointTarget").value;
	    const selectedOption=document.querySelector('input[name="setPointRelativity"]:checked');
		const id=selectedOption.id;
		
		var plainJSON='{';
		plainJSON+='"targetPosition":'+target+',';
		if (id=='Relative') {
	      plainJSON += '"type": "RELATIVE"}';
	    }else{
		  plainJSON += '"type": "ABSOLUTE"}';
		}

		fetch("/setPoint", {method: "POST",body: plainJSON});
	    setTimeout(() => {location.reload();}, 100);
	  }

	  function jog(){
	  	var velocity = document.getElementById("setVelocity").value;
		
		var plainJSON='{';
		plainJSON+='"velocity":'+velocity+'}';

		fetch("/jog", {method: "POST",body: plainJSON});
	    setTimeout(() => {location.reload();}, 100);
	  }

	  function putMqttConfig(){

		var brokerByte1 = document.getElementById("brokerIp1").value;
	    var brokerByte2 = document.getElementById("brokerIp2").value;
	    var brokerByte3 = document.getElementById("brokerIp3").value;
	    var brokerByte4 = document.getElementById("brokerIp4").value;

		var brokerPort = document.getElementById("brokerPort").value; 

		const selectedMqttEnabled=document.querySelector('input[name="mqttEnabledRadio"]:checked');
		const selectedMqttEnabledId=selectedMqttEnabled.id;

		var boardNumber = document.getElementById("boardNumber").value;

		const selectedPulseFeedbackEnabled=document.querySelector('input[name="mqttPulseFbRadio"]:checked');
		const selectedPulseFeedbackEnabledId=selectedPulseFeedbackEnabled.id;

		const selectedEncoderFeedbackEnabled=document.querySelector('input[name="mqttEncoderFbRadio"]:checked');
		const selectedEncoderFeedbackEnabledId=selectedEncoderFeedbackEnabled.id;

		const selectedSetpointFeedbackEnabled=document.querySelector('input[name="mqttSetpointFbRadio"]:checked');
		const selectedSetpointFeedbackEnabledId=selectedSetpointFeedbackEnabled.id;

		const selectedTimestampFeedbackEnabled=document.querySelector('input[name="mqttTimestampFbRadio"]:checked');
		const selectedTimestampFeedbackEnabledId=selectedTimestampFeedbackEnabled.id;

		const selectedErrorFeedbackEnabled=document.querySelector('input[name="mqttErrorFbRadio"]:checked');
		const selectedErrorFeedbackEnabledId=selectedErrorFeedbackEnabled.id;

		var publishPeriod = document.getElementById("publishPeriod").value; 

		var username = document.getElementById("mqttUsername").value; 
		var password = document.getElementById("mqttPassword").value;

		var plainJSON='{';

		if (brokerByte1 !== "" && brokerByte2 !== "" && brokerByte3 !== "" && brokerByte4 !== "") {
	      plainJSON += '"brokerIp":"' + brokerByte1 + '.' + brokerByte2 + '.'+ brokerByte3 + '.' + brokerByte4 + '",';
	    }

		if (brokerPort !== ""){
		  plainJSON += '"brokerPort":'+brokerPort+',';
		}

		if (selectedMqttEnabledId == 'mqttEnabled'){
		  plainJSON += '"mqttEnabled":true,';
		} else {
		  plainJSON += '"mqttEnabled":false,';
		}

		if (boardNumber !== ""){
		  plainJSON += '"boardNumber":'+boardNumber+',';
		}

		if (selectedPulseFeedbackEnabledId == 'mqttPulseFbEnabled'){
		  plainJSON += '"pulseFeedbackEnabled":true,';
		} else {
		  plainJSON += '"pulseFeedbackEnabled":false,';
		}

		if (selectedEncoderFeedbackEnabledId == 'mqttEncoderFbEnabled'){
		  plainJSON += '"encoderFeedbackEnabled":true,';
		} else {
		  plainJSON += '"encoderFeedbackEnabled":false,';
		}
		
		if (selectedSetpointFeedbackEnabledId == 'mqttSetpointFbEnabled'){
		  plainJSON += '"setPointFeedbackEnabled":true,';
		} else {
		  plainJSON += '"setPointFeedbackEnabled":false,';
		}

		if (selectedTimestampFeedbackEnabledId == 'mqttTimestampFbEnabled'){
		  plainJSON += '"timeStampFeedbackEnabled":true,';
		} else {
		  plainJSON += '"timeStampFeedbackEnabled":false,';
		}

		if (selectedErrorFeedbackEnabledId == 'mqttErrorFbEnabled'){
		  plainJSON += '"errorFeedbackEnabled":true,';
		} else {
		  plainJSON += '"errorFeedbackEnabled":false,';
		}

		if (publishPeriod !== ""){
		  plainJSON += '"publishPeriod":'+publishPeriod+',';
		}

		if (username !== ""){
		  plainJSON += '"username":"'+username+'",';
		}

		if (password !== ""){
		  plainJSON += '"password":"'+password+'",';
		}

		plainJSON=plainJSON.slice(0,-1);
	    plainJSON += '}';
	    fetch("/mqttConfig", {method: "PUT",body: plainJSON});
	    setTimeout(() => {location.reload();}, 100);
	  }

	  function reboot(){
	  	fetch("/reboot", {method: "POST",body:""});
	    setTimeout(() => {location.reload();}, 2000);
	  }
    </script>
)html"