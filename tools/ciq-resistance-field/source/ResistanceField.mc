import Toybox.WatchUi;
import Toybox.Activity;

// Displays Peloton resistance % from the CPS Pedal Power Balance field.
// The ESP32 firmware stuffs resistance (0-100) into the balance byte as
// resistance*2 (CPS 0.5% resolution), so Garmin delivers it back as 0.0-100.0.
class ResistanceField extends WatchUi.SimpleDataField {

    function initialize() {
        SimpleDataField.initialize();
        label = "Resist %";
    }

    function compute(info) {
        var bal = info.pedalPowerBalance;
        if (bal == null) { return "--"; }
        return bal.toNumber();
    }
}
