function timedRefresh() {
    var timer = setInterval(function() {
        eel.get_python_variable()(call_Back);
    }, 100);
 };
timedRefresh();

const svgMotorPos = document.getElementById("bldcSetPosition");
const svgMotorPower = document.getElementById("bldcSetPower");

function call_Back(output){  
    var partsArray = output.split(',');

    if(partsArray[0] == 0){
        document.getElementById("buttonMode1").style.backgroundColor="#52754a";
        document.getElementById("buttonMode2").style.backgroundColor="#803b3b";
    } else if (partsArray[0] == 1){
        document.getElementById("buttonMode1").style.backgroundColor="#803b3b";
        document.getElementById("buttonMode2").style.backgroundColor="#52754a";
    }

    var bldcAngle = Math.round(partsArray[1] / 11.406);
    document.getElementById("bldcPosition").innerHTML = bldcAngle + "°";
    var msg = 'rotate(' + bldcAngle + 'deg)'
    document.getElementById("motorPosition").style.transformOrigin = "50% 50%";
    document.getElementById("motorPosition").style.transform = msg;
    document.getElementById("bldcPwmU").innerHTML = partsArray[2]; 
    pwmU = Math.round(partsArray[2]) / 10;
    document.getElementById("pwmU").style.transform = "scale(" + pwmU  + "%, 100%)";
    document.getElementById("bldcPwmV").innerHTML = partsArray[3]; 
    pwmV = Math.round(partsArray[3]) / 10;
    document.getElementById("pwmV").style.transform = "scale(" + pwmV  + "%, 100%)";
    document.getElementById("bldcPwmW").innerHTML = partsArray[4]; 
    pwmW = Math.round(partsArray[4]) / 10;
    document.getElementById("pwmW").style.transform = "scale(" + pwmW  + "%, 100%)";

    var bldcExpectedAngle = Math.round(partsArray[5] / 11.406);
    var msg = 'rotate(' + bldcExpectedAngle + 'deg)'
    document.getElementById("motorExpectedPosition").style.transformOrigin = "50% 50%";
    document.getElementById("motorExpectedPosition").style.transform = msg;

    var rect = svgMotorPos.getBoundingClientRect();
    var pointerPos = Math.round((rect.width / 4095) * partsArray[5]);
    document.getElementById("posPointer").style.transform = "translate(" + pointerPos + "px, 0px)";
    document.getElementById("bldcExpectedPositionLabel").innerHTML = bldcExpectedAngle + "°";

    var rect2 = svgMotorPower.getBoundingClientRect();
    var pointerPower = Math.round((rect2.width / 100) * partsArray[6]);
    document.getElementById("powerPointer").style.transform = "translate(" + pointerPower + "px, 0px)";
    document.getElementById("bldcExpectedPowerLabel").innerHTML = partsArray[6] + "%";

    actualPower = Math.round(partsArray[7]);
    document.getElementById("actualPower").style.transform = "scale(" + actualPower  + "%, 100%)";
} 


const positionSvgHandler = (svgMotorPos, x, y) => {  //position
    let p = new DOMPoint(x, y);
    return p.matrixTransform(svgMotorPos.getScreenCTM().inverse());
};
svgMotorPos.addEventListener('click', e => {
    let p = positionSvgHandler(e.target, e.clientX, e.clientY);
    var rect = svgMotorPos.getBoundingClientRect();
    var newMotorPos = Math.round((359 / rect.width) * p.x);
    eel.set_motor_position(newMotorPos)();
});


const powerSvgHandler = (svgMotorPower, x, y) => { //power
    let p = new DOMPoint(x, y);
    return p.matrixTransform(svgMotorPower.getScreenCTM().inverse());
};
svgMotorPower.addEventListener('click', e => {
    let p = powerSvgHandler(e.target, e.clientX, e.clientY);
    var rect = svgMotorPower.getBoundingClientRect();
    var newMotorPower = Math.round((100 / rect.width) * p.x);
    eel.set_motor_power(newMotorPower)();
});

function set_mode(mode){
    eel.set_mode(mode)();
}
