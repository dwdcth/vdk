package ffmpeg

import (
	"github.com/dwdcth/ffmpeg-go/v7/ffcommon"
	"github.com/dwdcth/ffmpeg-go/v7/libavcodec"
	"github.com/dwdcth/ffmpeg-go/v7/libavutil"
	"runtime"
)

//const (
//	QUIET   = int(C.AV_LOG_QUIET)
//	PANIC   = int(C.AV_LOG_PANIC)
//	FATAL   = int(C.AV_LOG_FATAL)
//	ERROR   = int(C.AV_LOG_ERROR)
//	WARNING = int(C.AV_LOG_WARNING)
//	INFO    = int(C.AV_LOG_INFO)
//	VERBOSE = int(C.AV_LOG_VERBOSE)
//	DEBUG   = int(C.AV_LOG_DEBUG)
//	TRACE   = int(C.AV_LOG_TRACE)
//)

func HasEncoder(name string) bool {

	return libavcodec.AvcodecFindEncoderByName(name) != nil
}

func HasDecoder(name string) bool {
	return libavcodec.AvcodecFindDecoderByName(name) != nil
}

//func EncodersList() []string
//func DecodersList() []string

func SetLogLevel(level int) {
	libavutil.AvLogSetLevel(ffcommon.FInt(level))
}

/*

static inline int avcodec_profile_name_to_int(AVCodec *codec, const char *name) {
	const AVProfile *p;
	for (p = codec->profiles; p != NULL && p->profile != FF_PROFILE_UNKNOWN; p++)
		if (!strcasecmp(p->name, name))
			return p->profile;
	return FF_PROFILE_UNKNOWN;
}
*/

type ffctx struct {
	codecCtx *libavcodec.AVCodecContext
	codec    *libavcodec.AVCodec
	frame    *libavutil.AVFrame
	options  *libavutil.AVDictionary
	profile  ffcommon.FInt
}

func newFFCtxByCodec(codec *libavcodec.AVCodec) (ff *ffctx, err error) {
	ff = &ffctx{}
	ff.codec = codec
	ff.codecCtx = codec.AvcodecAllocContext3()
	//ff.profile = C.FF_PROFILE_UNKNOWN
	runtime.SetFinalizer(ff, freeFFCtx) //垃圾回收器回收之前执行一些清理操作
	return
}

func freeFFCtx(ff *ffctx) {

	if ff.frame != nil {
		libavutil.AvFrameFree(&ff.frame)
		ff.frame = nil
	}
	if ff.codecCtx != nil {
		ff.codecCtx.AvcodecClose()
		ff.codecCtx = nil
	}
	if ff.options != nil {
		libavutil.AvDictFree(&ff.options)
		ff.options = nil
	}
}
