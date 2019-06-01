# 32u4 Ubox 6m GPS TTN Mapper

## Install

- Copy the content of `libraries` folder in the arduino ide libraries folder
- Insert your `Network Session Key (MSB)`, `Application Session Key (MSB)`, `Device Address (MSB)`

## Decoder

```js
function Decoder(bytes, port) {
  // Decode an uplink message from a buffer
  // (array) of bytes to an object of fields.
  var decoded = {};

  if (port === 1) {
    var i = 0;
    decoded.latitude = parseInt((bytes[i++]<<24) + (bytes[i++]<<16) + (bytes[i++]<<8) + bytes[i++]) / 10000000;
    decoded.longitude = parseInt((bytes[i++]<<24) + (bytes[i++]<<16) + (bytes[i++]<<8) + bytes[i++]) / 10000000;
    decoded.altitude = parseInt((bytes[i++]<<8) + bytes[i++]) / 100;
    decoded.sats = (bytes[i++]<<8) + bytes[i++];
  }

  return decoded;
}
```

