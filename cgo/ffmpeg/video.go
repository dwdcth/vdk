package ffmpeg

import (
	"errors"
	"fmt"
	"github.com/dwdcth/ffmpeg-go/v7/ffcommon"
	"github.com/dwdcth/ffmpeg-go/v7/libavcodec"
	"github.com/dwdcth/ffmpeg-go/v7/libavutil"
	"github.com/dwdcth/vdk/av"
	"github.com/dwdcth/vdk/codec/h264parser"
	"image"
	"runtime"
	"unsafe"
)

type VideoDecoder struct {
	ff        *ffctx
	Extradata []ffcommon.FUint8T
	width     int
	height    int
}

func (self *VideoDecoder) Setup(width, height int) (err error) {
	ff := self.ff
	if len(self.Extradata) > 0 {
		ff.codecCtx.Extradata = &self.Extradata[0]
		ff.codecCtx.ExtradataSize = ffcommon.FInt(len(self.Extradata))
	}
	self.width = width
	self.height = height

	if ret := ff.codecCtx.AvcodecOpen2(ff.codec, nil); ret != 0 {
		err = fmt.Errorf("ffmpeg: decoder: avcodec_open2 failed %s", libavutil.AvErr2str(ret))
		return
	}
	return
}

func fromCPtr(buf unsafe.Pointer, size int) (ret []uint8) {

	//hdr := (*reflect.SliceHeader)((unsafe.Pointer(&ret)))
	//hdr.Cap = size
	//hdr.Len = size
	//hdr.Data = uintptr(buf)
	ret = unsafe.Slice((*uint8)(buf), size)
	return
}

type VideoFrame struct {
	Image image.YCbCr
	frame *libavutil.AVFrame
}

func (self *VideoFrame) Free() {
	self.Image = image.YCbCr{}
	libavutil.AvFrameFree(&self.frame)
}

//     av_dict_set(&options,"list_devices","true",0);
//  avformat_open_input(&pFormatCtx,"video=dummy",iformat,&options);

// ret = av_opt_set(c->priv_data, "profile", "aac_low", 0);
func freeVideoFrame(self *VideoFrame) {
	self.Free()
}

func (self *VideoDecoder) Decode(pkt []byte) (img *VideoFrame, err error) {
	ff := self.ff
	var ret ffcommon.FInt = 0

	frame := libavutil.AvFrameAlloc()

	var avPkt *libavcodec.AVPacket
	if pkt != nil {
		avPkt.Size = ffcommon.FUint(len(pkt))
		//avPkt.AvNewPacket(ffcommon.FInt(self.width * self.height))
		avPkt.Data = (*ffcommon.FUint8T)(&pkt[0]) //分配一个packet
	} else {
		avPkt = nil
	}

	ret = ff.codecCtx.AvcodecSendPacket(avPkt)
	if ret != 0 {
		err = fmt.Errorf("avcodec_send_packet failed %d", ret)
		return
	}
	ret = ff.codecCtx.AvcodecReceiveFrame(frame)
	if ret != 0 {
		err = fmt.Errorf("avcodec_receive_frame failed %d", ret)
		return
	}

	w := int(frame.Width)
	h := int(frame.Height)
	ys := int(frame.Linesize[0])
	cs := int(frame.Linesize[1])

	img = &VideoFrame{Image: image.YCbCr{
		Y:              fromCPtr(unsafe.Pointer(frame.Data[0]), ys*h),
		Cb:             fromCPtr(unsafe.Pointer(frame.Data[1]), cs*h/2),
		Cr:             fromCPtr(unsafe.Pointer(frame.Data[2]), cs*h/2),
		YStride:        ys,
		CStride:        cs,
		SubsampleRatio: image.YCbCrSubsampleRatio420,
		Rect:           image.Rect(0, 0, w, h),
	}, frame: frame}
	runtime.SetFinalizer(img, freeVideoFrame)

	return
}

func (self *VideoDecoder) DecodeSingle(pkt []byte) (img *VideoFrame, err error) {
	return self.Decode(nil)
}

func NewVideoDecoder(stream av.CodecData) (dec *VideoDecoder, err error) {
	_dec := &VideoDecoder{}
	video, ok := stream.(av.VideoCodecData)
	if !ok {
		return nil, errors.New("NewVideoDecoder failed: not a video codec data")
	}
	var id uint32

	switch video.Type() {
	case av.H264:
		h264 := stream.(h264parser.CodecData)
		_dec.Extradata = h264.AVCDecoderConfRecordBytes()
		id = libavcodec.AV_CODEC_ID_H264

	default:
		err = fmt.Errorf("ffmpeg: NewVideoDecoder codec=%v unsupported", stream.Type())
		return
	}

	c := libavcodec.AvcodecFindDecoder(libavcodec.AVCodecID(id))
	if c == nil || c.Type != libavutil.AVMEDIA_TYPE_VIDEO {
		err = fmt.Errorf("ffmpeg: cannot find video decoder codecId=%d", id)
		return
	}

	if _dec.ff, err = newFFCtxByCodec(c); err != nil {
		return
	}
	if err = _dec.Setup(video.Width(), video.Height()); err != nil {
		return
	}

	dec = _dec
	return
}
