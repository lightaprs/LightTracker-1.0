function Bytes2Float32(bytes) {
    var sign = (bytes & 0x80000000) ? -1 : 1;
    var exponent = ((bytes >> 23) & 0xFF) - 127;
    var significand = (bytes & ~(-1 << 23));

    if (exponent == 128) 
        return sign * ((significand) ? Number.NaN : Number.POSITIVE_INFINITY);

    if (exponent == -127) {
        if (significand == 0) return sign * 0.0;
        exponent = -126;
        significand /= (1 << 22);
    } else significand = (significand | (1 << 23)) / (1 << 23);

    return sign * significand * Math.pow(2, exponent);
}

function Decoder(bytes, port) {
	var latitude = bytes[3] << 24 | bytes[2] << 16 | bytes[1] << 8 | bytes[0];
	var longitude = bytes[7] << 24 | bytes[6] << 16 | bytes[5] << 8 | bytes[4];
	var altitude = bytes[11] << 24 | bytes[10] << 16 | bytes[9] << 8 | bytes[8];
	var accuracy = bytes[15] << 24 | bytes[14] << 16 | bytes[13] << 8 | bytes[12];
	
	return{
		latitude  : Bytes2Float32(latitude),
		longitude : Bytes2Float32(longitude),
		altitude  : Math.round(Bytes2Float32(altitude)),
		accuracy  : Bytes2Float32(altitude),
	};
}