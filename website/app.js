function alertButton(number) {
    alert("Some visualization.");
}

function getNumber() {
    var number = document.getElementById('numberInput').value;
    console.log("User entered number: " + number);
    localStorage.setItem('velocityNumber', number);
    alert("Output value: " + number);
}