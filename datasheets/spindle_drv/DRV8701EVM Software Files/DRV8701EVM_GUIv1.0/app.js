/*
 * This file is provided for custom JavaScript logic that your HTML files might need.
 * GUI Composer includes this JavaScript file by default within HTML pages authored in GUI Composer.
 */
require(["dojo/ready"], function(ready){
     ready(function(){
         // logic that requires that Dojo is fully initialized should go here

     });
});

function Invert(valueFromTarget) {
	return !valueFromTarget;
}

function DecToHex(valueFromTarget) {
	return valueFromTarget.toString(16);
}

function HexToDec(valueToTarget) {
	return parseInt(valueToTarget, 16);
}

function RoundTo2Dec(valueFromTarget) {
	return Math.round(valueFromTarget * 100) / 100;
}

function VrefPerToFloat(valueFromTarget) {
	return (valueFromTarget / 100) * 3.3;
}

function VrefFloatToPer(valueToTarget) {
	return (valueToTarget / 3.3) * 100;
}

function VrefToIlim(valueFromTarget) {
	return Math.round((((valueFromTarget / 100) * 3.3) / (20 * 0.01)) * 100) / 100;
}

function SOToCurrent(valueFromTarget) {
	return Math.round(((((valueFromTarget / 1023) * 3.3 * ((10 + 7.5)/10)) / 20) / 0.01) * 100) / 100;
}