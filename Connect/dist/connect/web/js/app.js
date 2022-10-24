function timedRefresh() {
    var timer = setInterval(function() {
        eel.get_python_variable()(call_Back);
    }, 100);
 };
timedRefresh();

function call_Back(output){  
    var partsArray = output.split(',');
    var bldcAngle = Math.round(partsArray[0] / 11.406);
    document.getElementById("bldcPosition").innerHTML = bldcAngle + "°";
    var msg = 'rotate(' + bldcAngle + 'deg)'
    document.getElementById("motorPosition").style.transformOrigin = "50% 50%";
    document.getElementById("motorPosition").style.transform = msg;
    document.getElementById("bldcPwmU").innerHTML = partsArray[1]; 
    pwmU = Math.round(partsArray[1]) / 10;
    document.getElementById("pwmU").style.transform = "scale(" + pwmU  + "%, 100%)";
    document.getElementById("bldcPwmV").innerHTML = partsArray[2]; 
    pwmV = Math.round(partsArray[2]) / 10;
    document.getElementById("pwmV").style.transform = "scale(" + pwmV  + "%, 100%)";
    document.getElementById("bldcPwmW").innerHTML = partsArray[3]; 
    pwmW = Math.round(partsArray[3]) / 10;
    document.getElementById("pwmW").style.transform = "scale(" + pwmW  + "%, 100%)";

    var bldcExpectedAngle = Math.round(partsArray[4] / 11.406);
    var msg = 'rotate(' + bldcExpectedAngle + 'deg)'
    document.getElementById("motorExpectedPosition").style.transformOrigin = "50% 50%";
    document.getElementById("motorExpectedPosition").style.transform = msg;
    document.getElementById("bldcExpectedPositionLabel").innerHTML = bldcExpectedAngle + "°";
} 

const svg = document.getElementById("bldcSetPosition");

const toSVGPoint = (svg, x, y) => {
    let p = new DOMPoint(x, y);
    return p.matrixTransform(svg.getScreenCTM().inverse());
};
  
svg.addEventListener('click', e => {
    let p = toSVGPoint(e.target, e.clientX, e.clientY);
    print.textContent = `x: ${p.x} - y: ${p.y}`;
    var rect = svg.getBoundingClientRect();
    var newMotorPos = Math.round((359 / rect.width) * p.x);
    var msg = "translate(" + (p.x-2) + "px, 0px)";
    console.log(msg);
    document.getElementById("posPointer").style.transform = msg;
    eel.set_motor_position(newMotorPos)();
});