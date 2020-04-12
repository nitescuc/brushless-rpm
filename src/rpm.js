const Gpio = require('pigpio').Gpio;
const KalmanFilter = require('kalmanjs');

class RpmReader {
    constructor(config) {
        this.config = config || {};
        this.config.minValue = this.config.minValue || 0;
        this.config.maxValue = this.config.maxValue || 10000;
        this.config.remapValues = this.config.remapValues || [0, 10000];

        if (this.config.useKalman) {
            this.kalmanFilter = new KalmanFilter();
        }

        this.inputPin = new Gpio(this.config.pin, {mode: Gpio.INPUT, alert: true});
        this.inputPin.pullUpDown(Gpio.PUD_UP);
        this.inputPin.on('alert', (level, tick) => {
            if (level == 1) {
              this.startTick = tick;
            } else {
              const endTick = tick;
              const diff = (endTick >> 0) - (this.startTick >> 0); // Unsigned 32 bit arithmetic

              this.value = this.remap(diff);
              
              if (this.config.callback && this.isDifferent(this.lastValue, this.value)) this.config.callback(this, this.value);

              this.lastValue = this.value;
            }
        });

        if (this.config.powerPin) {
            this.powerPin = new Gpio(this.config.powerPin, {mode: Gpio.OUTPUT});
            this.powerPin.digitalWrite(1);         
        }

    }
    getValue() {
        return this.value;
    }
    isDifferent(lastValue, value) {
        return Math.abs((lastValue || 0) - value) > (this.config.sensitivity || 0);
    }
    remap(value) {
        const remap = this.config.remapValues;
        if (!remap) return value;

        if (value < this.config.minValue) return remap[0];
        if (value > this.config.maxValue) return remap[1];

        const X_range = this.config.maxValue - this.config.minValue;
        const Y_range = remap[1] - remap[0];
        const XY_ratio = X_range/Y_range
    
        let remaped = ((value - X_range) / XY_ratio + Math.min(...remap));

        if (this.kalmanFilter) remaped = this.kalmanFilter(remaped);

        return remaped;
    }
}

module.exports = {
    RpmReader
}