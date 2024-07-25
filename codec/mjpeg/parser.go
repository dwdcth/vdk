package mjpeg

import "github.com/dwdcth/vdk/av"

type CodecData struct {
}

func (d CodecData) Type() av.CodecType {
	return av.MJPEG
}
