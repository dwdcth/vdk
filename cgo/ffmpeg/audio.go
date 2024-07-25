package ffmpeg

import (
	"fmt"
	"github.com/dwdcth/ffmpeg-go/v7/ffcommon"
	"github.com/dwdcth/ffmpeg-go/v7/libavcodec"
	"github.com/dwdcth/ffmpeg-go/v7/libavutil"
	"github.com/dwdcth/ffmpeg-go/v7/libswresample"
	"runtime"
	"strconv"
	"time"
	"unsafe"

	"github.com/dwdcth/vdk/av"
	"github.com/dwdcth/vdk/av/avutil"
	"github.com/dwdcth/vdk/codec/aacparser"
)

const debug = false

// ret = av_channel_layout_from_mask(&c->ch_layout, AV_CH_LAYOUT_STEREO);
type Resampler struct {
	inSampleFormat, OutSampleFormat   av.SampleFormat
	inChannelLayout, OutChannelLayout av.ChannelLayout
	inSampleRate, OutSampleRate       int
	avr                               *libswresample.SwrContext
}

/*
SwrContext *swr = swr_alloc();
av_opt_set_int(swr, "in_sample_rate", input_sample_rate, 0);
av_opt_set_int(swr, "out_sample_rate", output_sample_rate, 0);
// 其他参数设置...
swr_init(swr);

int output_samples = swr_get_out_samples(swr, input_samples);

swr_convert(swr, output_buffer, output_samples, (const uint8_t **)input_buffer, input_samples);

swr_free(&swr);
*/

func (self *Resampler) Resample(in av.AudioFrame) (out av.AudioFrame, err error) {
	formatChange := in.SampleRate != self.inSampleRate || in.SampleFormat != self.inSampleFormat || in.ChannelLayout != self.inChannelLayout

	var flush av.AudioFrame
	var ret ffcommon.FInt

	if formatChange {
		if self.avr != nil {
			outChannels := self.OutChannelLayout.Count()
			if !self.OutSampleFormat.IsPlanar() {
				outChannels = 1
			}
			outData := (**byte)(unsafe.Pointer(libavutil.AvCalloc(uint64(outChannels), 8)))
			outSampleCount := self.avr.SwrGetOutSamples(ffcommon.FInt(in.SampleCount))
			outLinesize := outSampleCount * ffcommon.FInt(self.OutSampleFormat.BytesPerSample())
			flush.Data = make([][]byte, outChannels)

			flush.ChannelLayout = self.OutChannelLayout
			flush.SampleFormat = self.OutSampleFormat
			flush.SampleRate = self.OutSampleRate

			//swr_ctx.SwrConvert(convert_data, pCodecCtx.FrameSize,
			//	(**byte)(unsafe.Pointer(&pFrame.Data)),
			//	pFrame.NbSamples)

			convertSamples := self.avr.SwrConvert(outData, outLinesize,
				(**byte)(unsafe.Pointer(&in.Data)), ffcommon.FInt(in.SampleCount))

			if convertSamples < 0 {
				err = fmt.Errorf("ffmpeg: avresample_convert_frame failed")
				return
			}
			flush.SampleCount = int(convertSamples)
			if ret < outSampleCount {
				for i := 0; i < outChannels; i++ {
					flush.Data[i] = flush.Data[i][:flush.SampleCount*self.OutSampleFormat.BytesPerSample()]
				}
			}

			//fmt.Println("flush:", "outSampleCount", outSampleCount, "convertSamples", convertSamples, "datasize", len(flush.Data[0]))
		} else {
			runtime.SetFinalizer(self, func(self *Resampler) {
				self.Close()
			})
		}
		self.avr.SwrClose()
		libswresample.SwrFree(&self.avr)
		self.avr = nil

		self.inSampleFormat = in.SampleFormat
		self.inSampleRate = in.SampleRate
		self.inChannelLayout = in.ChannelLayout

		var avr *libswresample.SwrContext = libswresample.SwrAlloc()
		avr.SwrAllocSetOpts(ffcommon.FInt64T(channelLayoutAV2FF(self.OutChannelLayout)),
			sampleFormatAV2FF(self.OutSampleFormat),
			ffcommon.FInt(self.OutSampleRate),

			ffcommon.FInt64T(channelLayoutAV2FF(self.inChannelLayout)),
			sampleFormatAV2FF(self.inSampleFormat),
			ffcommon.FInt(self.inSampleRate),
			0, uintptr(0))
		avr.SwrInit()

		self.avr = avr
	}

	var outChannels, outLinesize, outBytesPerSample int
	outSampleCount := self.avr.SwrGetOutSamples(ffcommon.FInt(in.SampleCount))
	if !self.OutSampleFormat.IsPlanar() {
		outChannels = 1
		outBytesPerSample = self.OutSampleFormat.BytesPerSample() * self.OutChannelLayout.Count()
		outLinesize = int(outSampleCount) * outBytesPerSample
	} else {
		outChannels = self.OutChannelLayout.Count()
		outBytesPerSample = self.OutSampleFormat.BytesPerSample()
		outLinesize = int(outSampleCount) * outBytesPerSample
	}

	out.ChannelLayout = self.OutChannelLayout
	out.SampleFormat = self.OutSampleFormat
	out.SampleRate = self.OutSampleRate
	outData := (**byte)(unsafe.Pointer(libavutil.AvCalloc(uint64(outChannels), 8)))
	convertSamples := self.avr.SwrConvert(outData, ffcommon.FInt(outLinesize),
		(**byte)(unsafe.Pointer(&in.Data)), ffcommon.FInt(in.SampleCount))

	if convertSamples < 0 {
		err = fmt.Errorf("ffmpeg: avresample_convert_frame failed")
		return
	}

	out.SampleCount = int(convertSamples)
	if convertSamples < outSampleCount {
		for i := 0; i < outChannels; i++ {
			flush.Data[i] = flush.Data[i][:flush.SampleCount*self.OutSampleFormat.BytesPerSample()]

		}
	}

	if flush.SampleCount > 0 {
		out = flush.Concat(out)
	}

	return
}

func (self *Resampler) Close() {
	self.avr.SwrClose()
	libswresample.SwrFree(&self.avr)
	self.avr = nil
}

type AudioEncoder struct {
	ff               *ffctx
	SampleRate       int
	Bitrate          int
	ChannelLayout    av.ChannelLayout
	SampleFormat     av.SampleFormat
	FrameSampleCount int
	framebuf         av.AudioFrame
	codecData        av.AudioCodecData
	resampler        *Resampler
}

func sampleFormatAV2FF(sampleFormat av.SampleFormat) (ffsamplefmt libavutil.AVSampleFormat) {
	switch sampleFormat {
	case av.U8:
		ffsamplefmt = libavutil.AV_SAMPLE_FMT_U8
	case av.S16:
		ffsamplefmt = libavutil.AV_SAMPLE_FMT_S16
	case av.S32:
		ffsamplefmt = libavutil.AV_SAMPLE_FMT_S32
	case av.FLT:
		ffsamplefmt = libavutil.AV_SAMPLE_FMT_FLT
	case av.DBL:
		ffsamplefmt = libavutil.AV_SAMPLE_FMT_DBL
	case av.U8P:
		ffsamplefmt = libavutil.AV_SAMPLE_FMT_U8P
	case av.S16P:
		ffsamplefmt = libavutil.AV_SAMPLE_FMT_S16P
	case av.S32P:
		ffsamplefmt = libavutil.AV_SAMPLE_FMT_S32P
	case av.FLTP:
		ffsamplefmt = libavutil.AV_SAMPLE_FMT_FLTP
	case av.DBLP:
		ffsamplefmt = libavutil.AV_SAMPLE_FMT_DBLP
	}
	return
}

func sampleFormatFF2AV(ffsamplefmt int32) (sampleFormat av.SampleFormat) {
	switch ffsamplefmt {
	case libavutil.AV_SAMPLE_FMT_U8: ///< unsigned 8 bits
		sampleFormat = av.U8
	case libavutil.AV_SAMPLE_FMT_S16: ///< signed 16 bits
		sampleFormat = av.S16
	case libavutil.AV_SAMPLE_FMT_S32: ///< signed 32 bits
		sampleFormat = av.S32
	case libavutil.AV_SAMPLE_FMT_FLT: ///< float
		sampleFormat = av.FLT
	case libavutil.AV_SAMPLE_FMT_DBL: ///< double
		sampleFormat = av.DBL
	case libavutil.AV_SAMPLE_FMT_U8P: ///< unsigned 8 bits, planar
		sampleFormat = av.U8P
	case libavutil.AV_SAMPLE_FMT_S16P: ///< signed 16 bits, planar
		sampleFormat = av.S16P
	case libavutil.AV_SAMPLE_FMT_S32P: ///< signed 32 bits, planar
		sampleFormat = av.S32P
	case libavutil.AV_SAMPLE_FMT_FLTP: ///< float, planar
		sampleFormat = av.FLTP
	case libavutil.AV_SAMPLE_FMT_DBLP: ///< double, planar
		sampleFormat = av.DBLP
	}
	return
}

func (self *AudioEncoder) SetSampleFormat(fmt av.SampleFormat) (err error) {
	self.SampleFormat = fmt
	return
}

func (self *AudioEncoder) SetSampleRate(rate int) (err error) {
	self.SampleRate = rate
	return
}

func (self *AudioEncoder) SetChannelLayout(ch av.ChannelLayout) (err error) {
	self.ChannelLayout = ch
	return
}

func (self *AudioEncoder) SetBitrate(bitrate int) (err error) {
	self.Bitrate = bitrate
	return
}

func (self *AudioEncoder) SetOption(key string, val interface{}) (err error) {
	ff := self.ff

	if key == "profile" {
		libavutil.AvOptSet(ff.codecCtx.PrivData, key, val.(string), 0)
		return
	}
	libavutil.AvOptSet(uintptr(unsafe.Pointer(ff.options)), key, val.(string), 0)
	return
}

func (self *AudioEncoder) GetOption(key string, val interface{}) (err error) {
	ff := self.ff
	var dst_data *ffcommon.FUint8T
	ret := libavutil.AvOptGet(ff.codecCtx.PrivData, key, 0, &dst_data)
	if ret < 0 {
		err = fmt.Errorf("ffmpeg: GetOption failed: `%s` not exists", key)
		return
	}
	option := ffcommon.GoStringFromBytePtr(dst_data)
	switch p := val.(type) {
	case *string:
		*p = option
	case *int:

		*p, _ = strconv.Atoi(option)
	default:
		err = fmt.Errorf("ffmpeg: GetOption failed: val must be *string or *int receiver")
		return
	}
	return
}

func (self *AudioEncoder) Setup() (err error) {
	ff := self.ff

	ff.frame = libavutil.AvFrameAlloc()

	if self.SampleFormat == av.SampleFormat(0) {
		//ff.codec.GetSampleFmt() //todo
		//self.SampleFormat = sampleFormatFF2AV()
	}

	//if self.Bitrate == 0 {
	//	self.Bitrate = 80000
	//}
	if self.SampleRate == 0 {
		self.SampleRate = 44100
	}
	if self.ChannelLayout == av.ChannelLayout(0) {
		self.ChannelLayout = av.CH_STEREO
	}
	ff.codecCtx = ff.codec.AvcodecAllocContext3()

	ff.codecCtx.SampleFmt = sampleFormatAV2FF(self.SampleFormat)
	ff.codecCtx.SampleRate = ffcommon.FInt(self.SampleRate)
	ff.codecCtx.BitRate = ffcommon.FInt64T(self.Bitrate)
	ff.codecCtx.ChannelLayout = channelLayoutAV2FF(self.ChannelLayout)
	ff.codecCtx.StrictStdCompliance = libavcodec.FF_COMPLIANCE_EXPERIMENTAL
	ff.codecCtx.Flags = libavcodec.AV_CODEC_FLAG_GLOBAL_HEADER
	ff.codecCtx.Profile = ff.profile

	if ff.codecCtx.AvcodecOpen2(ff.codec, nil) != 0 {
		err = fmt.Errorf("ffmpeg: encoder: avcodec_open2 failed")
		return
	}
	self.SampleFormat = sampleFormatFF2AV(int32(ff.codecCtx.SampleFmt))
	self.FrameSampleCount = int(ff.codecCtx.FrameSize)

	extradata := ffcommon.ByteSliceFromByteP(ff.codecCtx.Extradata, int(ff.codecCtx.ExtradataSize))

	switch ff.codecCtx.CodecId {
	case libavcodec.AV_CODEC_ID_AAC:
		if self.codecData, err = aacparser.NewCodecDataFromMPEG4AudioConfigBytes(extradata); err != nil {
			return
		}

	default:
		self.codecData = audioCodecData{
			channelLayout: self.ChannelLayout,
			sampleFormat:  self.SampleFormat,
			sampleRate:    self.SampleRate,
			codecId:       uint32(ff.codecCtx.CodecId),
			extradata:     extradata,
		}
	}

	return
}

func (self *AudioEncoder) prepare() (err error) {
	ff := self.ff

	if ff.frame == nil {
		if err = self.Setup(); err != nil {
			return
		}
	}

	return
}

func (self *AudioEncoder) CodecData() (codec av.AudioCodecData, err error) {
	if err = self.prepare(); err != nil {
		return
	}
	codec = self.codecData
	return
}

func (self *AudioEncoder) encodeOne(frame av.AudioFrame) (gotpkt bool, pkt []byte, err error) {
	if err = self.prepare(); err != nil {
		return
	}

	ff := self.ff

	cpkt := libavcodec.AvPacketAlloc()
	cgotpkt := 0
	audioFrameAssignToFF(frame, ff.frame)

	//if false {
	//	farr := []string{}
	//	for i := 0; i < len(frame.Data[0])/4; i++ {
	//		var f *float64 = (*float64)(unsafe.Pointer(&frame.Data[0][i*4]))
	//		farr = append(farr, fmt.Sprintf("%.8f", *f))
	//	}
	//	fmt.Println(farr)
	//}

	cerr := ff.codecCtx.AvcodecSendFrame(ff.frame)
	if cerr < 0 {
		err = fmt.Errorf("ffmpeg: avcodec_encode_audio2 failed: %d", cerr)
		return
	}
	cerr = ff.codecCtx.AvcodecReceivePacket(cpkt)
	if cerr < 0 {
		err = fmt.Errorf("ffmpeg: avcodec_encode_audio2 failed: %d", cgotpkt)
		return
	}

	gotpkt = true
	pkt = unsafe.Slice((*uint8)(cpkt.Data), cpkt.Size)
	libavcodec.AvPacketFree(&cpkt)

	if debug {
		fmt.Println("ffmpeg: Encode", frame.SampleCount, frame.SampleRate, frame.ChannelLayout, frame.SampleFormat, "len", len(pkt))
	}

	return
}

func (self *AudioEncoder) resample(in av.AudioFrame) (out av.AudioFrame, err error) {
	if self.resampler == nil {
		self.resampler = &Resampler{
			OutSampleFormat:  self.SampleFormat,
			OutSampleRate:    self.SampleRate,
			OutChannelLayout: self.ChannelLayout,
		}
	}
	if out, err = self.resampler.Resample(in); err != nil {
		return
	}
	return
}

func (self *AudioEncoder) Encode(frame av.AudioFrame) (pkts [][]byte, err error) {
	var gotpkt bool
	var pkt []byte

	if frame.SampleFormat != self.SampleFormat || frame.ChannelLayout != self.ChannelLayout || frame.SampleRate != self.SampleRate {
		if frame, err = self.resample(frame); err != nil {
			return
		}
	}

	if self.FrameSampleCount != 0 {
		if self.framebuf.SampleCount == 0 {
			self.framebuf = frame
		} else {
			self.framebuf = self.framebuf.Concat(frame)
		}
		for self.framebuf.SampleCount >= self.FrameSampleCount {
			frame := self.framebuf.Slice(0, self.FrameSampleCount)
			if gotpkt, pkt, err = self.encodeOne(frame); err != nil {
				return
			}
			if gotpkt {
				pkts = append(pkts, pkt)
			}
			self.framebuf = self.framebuf.Slice(self.FrameSampleCount, self.framebuf.SampleCount)
		}
	} else {
		if gotpkt, pkt, err = self.encodeOne(frame); err != nil {
			return
		}
		if gotpkt {
			pkts = append(pkts, pkt)
		}
	}

	return
}

func (self *AudioEncoder) Close() {
	freeFFCtx(self.ff)
	if self.resampler != nil {
		self.resampler.Close()
		self.resampler = nil
	}
}

func audioFrameAssignToAVParams(f *libavutil.AVFrame, frame *av.AudioFrame) {
	frame.SampleFormat = sampleFormatFF2AV(int32(f.Format))
	frame.ChannelLayout = channelLayoutFF2AV(f.ChannelLayout)
	frame.SampleRate = int(f.SampleRate)
}

func audioFrameAssignToAVData(f *libavutil.AVFrame, frame *av.AudioFrame) {
	frame.SampleCount = int(f.NbSamples)
	frame.Data = make([][]byte, int(f.Channels))
	for i := 0; i < int(f.Channels); i++ {
		frame.Data[i] = unsafe.Slice((*uint8)(f.Data[i]), f.Linesize[0])
	}

}

func audioFrameAssignToAV(f *libavutil.AVFrame, frame *av.AudioFrame) {
	audioFrameAssignToAVParams(f, frame)
	audioFrameAssignToAVData(f, frame)
}

func audioFrameAssignToFFParams(frame av.AudioFrame, f *libavutil.AVFrame) {
	f.Format = ffcommon.FInt(sampleFormatAV2FF(frame.SampleFormat))
	f.ChannelLayout = channelLayoutAV2FF(frame.ChannelLayout)
	f.SampleRate = ffcommon.FInt(frame.SampleRate)
	f.Channels = ffcommon.FInt(frame.ChannelLayout.Count())
}

func audioFrameAssignToFFData(frame av.AudioFrame, f *libavutil.AVFrame) {
	f.NbSamples = ffcommon.FInt(frame.SampleCount)
	for i := range frame.Data {
		fd := uintptr(unsafe.Pointer(f.Data[i]))
		gd := uintptr(frame.Data[i][0])
		*(*byte)(unsafe.Pointer(fd)) = *(*byte)(unsafe.Pointer(gd))

		f.Linesize[i] = int32(len(frame.Data[i]))
	}
}

func audioFrameAssignToFF(frame av.AudioFrame, f *libavutil.AVFrame) {
	audioFrameAssignToFFParams(frame, f)
	audioFrameAssignToFFData(frame, f)
}

func channelLayoutFF2AV(layout ffcommon.FUint64T) (channelLayout av.ChannelLayout) {
	if layout&libavutil.AV_CH_FRONT_CENTER != 0 {
		channelLayout |= av.CH_FRONT_CENTER
	}
	if layout&libavutil.AV_CH_FRONT_LEFT != 0 {
		channelLayout |= av.CH_FRONT_LEFT
	}
	if layout&libavutil.AV_CH_FRONT_RIGHT != 0 {
		channelLayout |= av.CH_FRONT_RIGHT
	}
	if layout&libavutil.AV_CH_BACK_CENTER != 0 {
		channelLayout |= av.CH_BACK_CENTER
	}
	if layout&libavutil.AV_CH_BACK_LEFT != 0 {
		channelLayout |= av.CH_BACK_LEFT
	}
	if layout&libavutil.AV_CH_BACK_RIGHT != 0 {
		channelLayout |= av.CH_BACK_RIGHT
	}
	if layout&libavutil.AV_CH_SIDE_LEFT != 0 {
		channelLayout |= av.CH_SIDE_LEFT
	}
	if layout&libavutil.AV_CH_SIDE_RIGHT != 0 {
		channelLayout |= av.CH_SIDE_RIGHT
	}
	if layout&libavutil.AV_CH_LOW_FREQUENCY != 0 {
		channelLayout |= av.CH_LOW_FREQ
	}
	return
}

func channelLayoutAV2FF(channelLayout av.ChannelLayout) (layout ffcommon.FUint64T) {
	if channelLayout&av.CH_FRONT_CENTER != 0 {
		layout |= libavutil.AV_CH_FRONT_CENTER
	}
	if channelLayout&av.CH_FRONT_LEFT != 0 {
		layout |= libavutil.AV_CH_FRONT_LEFT
	}
	if channelLayout&av.CH_FRONT_RIGHT != 0 {
		layout |= libavutil.AV_CH_FRONT_RIGHT
	}
	if channelLayout&av.CH_BACK_CENTER != 0 {
		layout |= libavutil.AV_CH_BACK_CENTER
	}
	if channelLayout&av.CH_BACK_LEFT != 0 {
		layout |= libavutil.AV_CH_BACK_LEFT
	}
	if channelLayout&av.CH_BACK_RIGHT != 0 {
		layout |= libavutil.AV_CH_BACK_RIGHT
	}
	if channelLayout&av.CH_SIDE_LEFT != 0 {
		layout |= libavutil.AV_CH_SIDE_LEFT
	}
	if channelLayout&av.CH_SIDE_RIGHT != 0 {
		layout |= libavutil.AV_CH_SIDE_RIGHT
	}
	if channelLayout&av.CH_LOW_FREQ != 0 {
		layout |= libavutil.AV_CH_LOW_FREQUENCY
	}
	return
}

type AudioDecoder struct {
	ff            *ffctx
	ChannelLayout av.ChannelLayout
	SampleFormat  av.SampleFormat
	SampleRate    int
	Extradata     []byte
}

func (self *AudioDecoder) Setup() (err error) {
	ff := self.ff

	ff.frame = libavutil.AvFrameAlloc()

	if len(self.Extradata) > 0 {
		ff.codecCtx.Extradata = &self.Extradata[0]
		ff.codecCtx.ExtradataSize = ffcommon.FInt(len(self.Extradata))
	}
	if debug {
		fmt.Println("ffmpeg: Decoder.Setup Extradata.len", len(self.Extradata))
	}

	ff.codecCtx.SampleRate = ffcommon.FInt(self.SampleRate)
	ff.codecCtx.ChannelLayout = channelLayoutAV2FF(self.ChannelLayout)
	ff.codecCtx.Channels = ffcommon.FInt(self.ChannelLayout.Count())

	if ff.codecCtx.AvcodecOpen2(ff.codec, nil) != 0 {
		err = fmt.Errorf("ffmpeg: decoder: avcodec_open2 failed")
		return
	}
	self.SampleFormat = sampleFormatFF2AV(int32(ff.codecCtx.SampleFmt))
	self.ChannelLayout = channelLayoutFF2AV(ff.codecCtx.ChannelLayout)
	if self.SampleRate == 0 {
		self.SampleRate = int(ff.codecCtx.SampleRate)
	}

	return
}

func (self *AudioDecoder) Decode(pkt []byte) (gotframe bool, frame av.AudioFrame, err error) {
	ff := self.ff
	var ret ffcommon.FInt = 0

	avframe := libavutil.AvFrameAlloc()

	var avPkt *libavcodec.AVPacket
	if pkt != nil {
		avPkt.Size = ffcommon.FUint(len(pkt))
		avPkt.Data = (*ffcommon.FUint8T)(&pkt[0]) //分配一个packet
	} else {
		avPkt = nil
	}

	ret = ff.codecCtx.AvcodecSendPacket(avPkt)
	if ret != 0 {
		err = fmt.Errorf("avcodec_send_packet failed %d", ret)
		return
	}
	ret = ff.codecCtx.AvcodecReceiveFrame(avframe)
	if ret != 0 {
		err = fmt.Errorf("avcodec_receive_frame failed %d", ret)
		return
	}
	gotframe = true
	frame.SampleRate = self.SampleRate
	audioFrameAssignToAV(avframe, &frame)

	return
}

func (self *AudioDecoder) Close() {
	freeFFCtx(self.ff)
}

func NewAudioEncoderByCodecType(typ av.CodecType) (enc *AudioEncoder, err error) {
	var id uint32

	switch typ {
	case av.AAC:
		id = libavcodec.AV_CODEC_ID_AAC

	default:
		err = fmt.Errorf("ffmpeg: cannot find encoder codecType=%d", typ)
		return
	}

	codec := libavcodec.AvcodecFindEncoder(libavcodec.AVCodecID(id))
	if codec == nil || codec.Type != libavutil.AVMEDIA_TYPE_AUDIO {
		err = fmt.Errorf("ffmpeg: cannot find audio encoder codecId=%d", id)
		return
	}

	_enc := &AudioEncoder{}
	if _enc.ff, err = newFFCtxByCodec(codec); err != nil {
		return
	}
	enc = _enc
	return
}

func NewAudioEncoderByName(name string) (enc *AudioEncoder, err error) {
	_enc := &AudioEncoder{}

	codec := libavcodec.AvcodecFindEncoderByName(name)
	if codec == nil || codec.Type != libavutil.AVMEDIA_TYPE_AUDIO {
		err = fmt.Errorf("ffmpeg: cannot find audio encoder name=%s", name)
		return
	}

	if _enc.ff, err = newFFCtxByCodec(codec); err != nil {
		return
	}
	enc = _enc
	return
}

func NewAudioDecoder(codec av.AudioCodecData) (dec *AudioDecoder, err error) {
	_dec := &AudioDecoder{}
	var id uint32

	switch codec.Type() {
	case av.AAC:
		if aaccodec, ok := codec.(aacparser.CodecData); ok {
			_dec.Extradata = aaccodec.MPEG4AudioConfigBytes()
			id = libavcodec.AV_CODEC_ID_AAC
		} else {
			err = fmt.Errorf("ffmpeg: aac CodecData must be aacparser.CodecData")
			return
		}

	case av.SPEEX:
		id = libavcodec.AV_CODEC_ID_SPEEX

	case av.PCM_MULAW:
		id = libavcodec.AV_CODEC_ID_PCM_MULAW

	case av.PCM_ALAW:
		id = libavcodec.AV_CODEC_ID_PCM_ALAW

	default:
		if ffcodec, ok := codec.(audioCodecData); ok {
			_dec.Extradata = ffcodec.extradata
			id = ffcodec.codecId
		} else {
			err = fmt.Errorf("ffmpeg: invalid CodecData for ffmpeg to decode")
			return
		}
	}

	coder := libavcodec.AvcodecFindDecoder(libavcodec.AVCodecID(id))
	if coder == nil || coder.Type != libavutil.AVMEDIA_TYPE_AUDIO {
		err = fmt.Errorf("ffmpeg: cannot find audio decoder id=%d", id)
		return
	}

	if _dec.ff, err = newFFCtxByCodec(coder); err != nil {
		return
	}

	_dec.SampleFormat = codec.SampleFormat()
	_dec.SampleRate = codec.SampleRate()
	_dec.ChannelLayout = codec.ChannelLayout()
	if err = _dec.Setup(); err != nil {
		return
	}

	dec = _dec
	return
}

type audioCodecData struct {
	codecId       uint32
	sampleFormat  av.SampleFormat
	channelLayout av.ChannelLayout
	sampleRate    int
	extradata     []byte
}

func (self audioCodecData) Type() av.CodecType {
	return av.MakeAudioCodecType(self.codecId)
}

func (self audioCodecData) SampleRate() int {
	return self.sampleRate
}

func (self audioCodecData) SampleFormat() av.SampleFormat {
	return self.sampleFormat
}

func (self audioCodecData) ChannelLayout() av.ChannelLayout {
	return self.channelLayout
}

func (self audioCodecData) PacketDuration(data []byte) (dur time.Duration, err error) {
	// TODO: implement it: ffmpeg get_audio_frame_duration
	err = fmt.Errorf("ffmpeg: cannot get packet duration")
	return
}

func AudioCodecHandler(h *avutil.RegisterHandler) {
	h.AudioDecoder = func(codec av.AudioCodecData) (av.AudioDecoder, error) {
		if dec, err := NewAudioDecoder(codec); err != nil {
			return nil, nil
		} else {
			return dec, err
		}
	}

	h.AudioEncoder = func(typ av.CodecType) (av.AudioEncoder, error) {
		if enc, err := NewAudioEncoderByCodecType(typ); err != nil {
			return nil, nil
		} else {
			return enc, err
		}
	}
}
