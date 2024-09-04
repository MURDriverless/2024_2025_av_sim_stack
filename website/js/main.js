function alertButton(number) {
    alert("Some visualization.");
}

function getNumber() {
    var number = document.getElementById("numberInput").value;
    console.log("User entered number: " + number);
    localStorage.setItem('velocityNumber', number);
    alert("Output value: " + number);
}

function getX() {
    var xVal = document.getElementById("xCoInput").value;
    console.log("User entered number: " + xVal);
    localStorage.setItem('xCoInput', xVal);
    alert("Output value: " + xVal);
}

function getY() {
    var yVal = document.getElementById("yCoInput").value;
    console.log("User entered number: " + yVal);
    localStorage.setItem('yCoInput', yVal);
    alert("Output value: " + yVal);
}
function getSteeringRate() {
    var steeringRate = document.getElementById("steeringRateInput").value;
    console.log("User entered number: " + steeringRate);
    localStorage.setItem('steeringRateInput', steeringRate);
    alert("Output value: " + steeringRate);
}
