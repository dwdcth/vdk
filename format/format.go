package format

import (
	"github.com/dwdcth/vdk/av/avutil"
	"github.com/dwdcth/vdk/format/aac"
	"github.com/dwdcth/vdk/format/flv"
	"github.com/dwdcth/vdk/format/mp4"
	"github.com/dwdcth/vdk/format/rtmp"
	"github.com/dwdcth/vdk/format/rtsp"
	"github.com/dwdcth/vdk/format/ts"
)

func RegisterAll() {
	avutil.DefaultHandlers.Add(mp4.Handler)
	avutil.DefaultHandlers.Add(ts.Handler)
	avutil.DefaultHandlers.Add(rtmp.Handler)
	avutil.DefaultHandlers.Add(rtsp.Handler)
	avutil.DefaultHandlers.Add(flv.Handler)
	avutil.DefaultHandlers.Add(aac.Handler)
}
