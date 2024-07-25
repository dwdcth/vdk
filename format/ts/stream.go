package ts

import (
	"time"

	"github.com/dwdcth/vdk/av"
	"github.com/dwdcth/vdk/format/ts/tsio"
)

type Stream struct {
	av.CodecData

	demuxer *Demuxer
	muxer   *Muxer

	pid        uint16
	streamId   uint8
	streamType uint8

	tsw          *tsio.TSWriter
	idx          int
	fps          uint
	iskeyframe   bool
	pts, dts, pt time.Duration
	data         []byte
	datalen      int
}
