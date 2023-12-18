<?php
	plugin_header();
	$m      =   ($PAGE == 'mb_clipper_mono') ? 'm' : 's';
?>

<p>
	This plugin allows to drive much more loudness by cutting extra peaks and reach extreme loudness
	levels at the output. The key features are following.
</p>
<p>
	<b>Classic</b> and <b>Linear Phase</b> crossover modes allow to bring more flexibility to the mastering process.
	<b>Low-pass</b> and <b>High-pass</b> pre-filters allow to cut unnecessary frequencies before processing.
	<b>Dithering</b> allows to add some dithering noise to the signal and make the final mix more detailed.
	<b>Loudness limiting</b> option allows to control the input loudness, per-band loudness and input loudness at the
	output clipper.
	<b>Overdrive protection</b> allows to add short-time compression to the signal to make clipping effect less noticeable.
	<b>Multiple sigmoid functions</b> allow to select the best sounding clipping funciton.
	<b>Input and output loudness measurments</b> allow to control loudness level of the signal in LUFS.
</p>

<p><b>Controls:</b></p>
<ul>
	<li>
		<b>Bypass</b> - bypass switch, when turned on (led indicator is shining), the output signal is similar to input signal. That does not mean
		that the plugin is not working.
	</li>
	<li><b>Mode</b> - crossover mode:</li>
	<ul>
		<li><b>Classic</b> - classic Linkwitz-Riley filters are used for splitting signal into multiple frequency bands.</li>
		<li><b>Linear Phase</b> - FFT transform is used to split audio signal into multiple bands, introduces additional latency.</li>
	</ul>
	<li><b>Slope</b> - the slope of crossover filters:</li>
	<ul>
		<li><b>LR4 (24 dB/oct)</b>.</li>
		<li><b>LR8 (48 dB/oct)</b>.</li>
		<li><b>LR12 (72 dB/oct)</b>.</li>
		<li><b>LR16 (96 dB/oct)</b>.</li>
	</ul>
	<li><b>Dither</b> - allows to enable dithering noise depending on the bitness of the desired output signal.</li>
	<li><b>Filters</b> - the button that enables drawing of crossover filter characteristics.</li>
	<li><b>Output clipper</b> - enables ouput clipper.</li>
	<li><b>LUFS Limit</b> button enables limiting of LUFS value at the input of the plugin</li>
	<li><b>LUFS Limit</b> knob allows to set maximum LUFS value of the input signal.</li>
</ul>

<p><b>Signal</b> section:</p>
<ul>
	<li><b>Input</b> - additional gain applied to the input signal.</li>
	<li><b>Output</b> - additional gain applied to the output signal.</li>
</ul>

<p><b>Global Threshold</b> section:</p>
<ul>
	<li><b>Boost</b> - allows to additionally boost the processed signal to compensate threshold level lower than 0 dB.</li>
	<li><b>Threshold</b> - allows to adjust lower than 0 dB clipping threshold.</li>
</ul>

<p><b>Analysis</b> section:</p>
<ul>
	<li><b>Reactivity</b> - the reactivity (smoothness) of the spectral analysis.</li>
	<li><b>Shift</b> - allows to adjust the overall gain of the analysis.</li>
	<li><b>FFT<?= $sm ?> In</b> - enables FFT curve graph of input signal on the spectrum graph.</li>
	<li><b>FFT<?= $sm ?> Out</b> - enables FFT curve graph of output signal on the spectrum graph.</li>
	<li><b>Filters</b> - enables drawing tranfer function of each sidechain filter on the spectrum graph.</li>
	<li><b>Surge</b> - enables surge protection mechanism.</li>
</ul>

<p>Meters:</p>
<ul>
	<li><b>LUFS</b> - input and output loudness meters.</li>
	<li><b>dB In</b> - the level meter of the input signal.</li>
	<li><b>dB Out</b> - the level meter of the output signal.</li>
	<li><b>LUFS Limit</b> - the amount of gain reduction applied to the input signal while limiting the loudness.</li>
</ul>

<p><b>Crossover</b> section:</p>
<ul>
	<li><b>HPF</b> combo allows to select the slope of high-pass pre-filter of the input signal.</li>
	<li><b>HPF</b> knob allows to adjust the cut-off frequency of the high-pass filter.</li>
	<li><b>Band solo</b> - allows to solo specific band.</li>
	<li><b>Band mute</b> - allows to mute specific band.</li>
	<li><b>Band knob</b> - allows to set the split frequency between bands.</li>
	<li><b>Band 4 act</b> - enables additional fourth band for processing.</li>
	<li><b>LPF</b> combo allows to select the slope of low-pass pre-filter of the input signal.</li>
	<li><b>LPF</b> knob allows to adjust the cut-off frequency of the low-pass filter.</li>
</ul>

<p><b>Band</b> tabs:</p>
<ul>
	<li><b>Preamp</b> - allows to control input gain level of the specific band.</li>
	<li><b>LUFS Limit</b> button - allows to enable loudness limiting for the specific band.</li>
	<li><b>LUFS Limit</b> knob - allows to set maximum allowed loudness of the signal at the input of the band.</li>
	<li><b>LUFS Limit</b> meter - the amount of gain reduction applied to the input signal of the band while reducing loudness.</li>
	<li><b>Act</b> - enables additional fourth band.</li>
	<li><b>Solo</b> - enables solo mode for the selected band.</li>
	<li><b>Mute</b> - mutes the selected band.</li>
	<li><b>ODP</b> - enabled overdrive protection compressor.</li>
	<li><b>Clipping</b> - enables clipping function applied to the signal.</li>
	<li><b>Log Scale</b> - switches clipping function graph representation in linear/logarithmic scale.</li>
	<li><b>Function</b> - clipping function</li>
	<li><b>ODP Thresh</b> - the threshold of the overdrive protection compressor.</li>
	<li><b>ODP Knee</b> - the knee of the overdrive protection compressor.</li>
	<li><b>ODP Meter</b> - the amount of gain reduction applied to the signal while compressing it's peaks.</li>
	<li><b>ODP Link</b> - knob that controls the side-chaining of the current band by the previous one.</li>
	<li><b>Clip Thresh</b> - the threshold of the clipping function. Signals below the threshold have constant amplification.</li>
	<li><b>Clip Pumping</b> - additional way to pump the loudness of the band by applying exra amplification and keeping peaks not greater than 0 dB.</li>
	<li><b>Clip Meter</b> - the amount of gain reduction applied at the clipping stage.</li>
	<li><b>Resonance</b> - sets up ODP compression time by selecting the dominating frequency in the signal.</li>
	<?php if ($m == 's') { ?>
	<li><b>Stereo Link</b> - allows to control how the left channel of ODP compressor affects the right channel and vice verse.</li>
	<?php } ?>
	<li><b>Makeup</b> - the output gain of the band</li>
	<li><b>Time Graph Gain</b> - the overall band gain reduction meter.</li>
	<li><b>Time Graph In</b> - the input signal meter of the band.</li>
	<li><b>Time Graph Out</b> - the output signal meter of the band.</li>
</ul>

<p><b>Output Clipper</b> tab:</p>
<ul>
	<li><b>LUFS Limit</b> button - allows to enable loudness limiting at the input of the output clipper.</li>
	<li><b>LUFS Limit</b> knob - allows to set maximum allowed loudness at the input of the output clipper.</li>
	<li><b>LUFS Limit</b> meter - the amount of gain reduction applied to the input of the output clipper signal while reducing loudness.</li>
	<li><b>Active</b> - enables output clipper.</li>
	<li><b>ODP</b> - enabled overdrive protection compressor.</li>
	<li><b>Clipping</b> - enables clipping function applied to the signal.</li>
	<li><b>Log Scale</b> - switches clipping function graph representation in linear/logarithmic scale.</li>
	<li><b>Function</b> - clipping function</li>
	<li><b>ODP Thresh</b> - the threshold of the overdrive protection compressor.</li>
	<li><b>ODP Knee</b> - the knee of the overdrive protection compressor.</li>
	<li><b>ODP Meter</b> - the amount of gain reduction applied to the signal while compressing it's peaks.</li>
	<li><b>Clip Thresh</b> - the threshold of the clipping function. Signals below the threshold have constant amplification.</li>
	<li><b>Clip Pumping</b> - additional way to pump the loudness of the band by applying exra amplification and keeping peaks not greater than 0 dB.</li>
	<li><b>Clip Meter</b> - the amount of gain reduction applied at the clipping stage.</li>
	<li><b>Reactivity</b> - sets up the reactivity of the ODP compressor.</li>
	<?php if ($m == 's') { ?>
	<li><b>Stereo Link</b> - allows to control how the left channel of ODP compressor affects the right channel and vice verse.</li>
	<?php } ?>
	<li><b>Time Graph Gain</b> - the overall band gain reduction meter.</li>
	<li><b>Time Graph In</b> - the input signal meter of the band.</li>
	<li><b>Time Graph Out</b> - the output signal meter of the band.</li>
</ul>




