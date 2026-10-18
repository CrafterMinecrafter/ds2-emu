#include "PluginProcessor.h"

namespace
{
    juce::ParameterID pid (const char* id) { return { id, 1 }; }

    juce::String knobText (float v, int) { return juce::String (v, 1); }
}

//==============================================================================
TwinModeDistProcessor::TwinModeDistProcessor()
    : AudioProcessor (BusesProperties()
                          .withInput  ("Input",  juce::AudioChannelSet::stereo(), true)
                          .withOutput ("Output", juce::AudioChannelSet::stereo(), true)),
      apvts (*this, nullptr, "PARAMS", createParameterLayout())
{
    dist    = apvts.getRawParameterValue ("dist");
    tone    = apvts.getRawParameterValue ("tone");
    level   = apvts.getRawParameterValue ("level");
    turbo   = apvts.getRawParameterValue ("turbo");
    inTrim  = apvts.getRawParameterValue ("input");
    outTrim = apvts.getRawParameterValue ("output");
    quality = apvts.getRawParameterValue ("quality");

    startTimerHz (5);
}

TwinModeDistProcessor::~TwinModeDistProcessor()
{
    stopTimer();
}

juce::AudioProcessorValueTreeState::ParameterLayout TwinModeDistProcessor::createParameterLayout()
{
    using namespace juce;
    AudioProcessorValueTreeState::ParameterLayout layout;

    auto knob = [] (const char* id, const char* name, float def)
    {
        return std::make_unique<AudioParameterFloat> (
            pid (id), name, NormalisableRange<float> (0.0f, 10.0f, 0.01f), def,
            AudioParameterFloatAttributes().withStringFromValueFunction (knobText));
    };

    layout.add (knob ("dist",  "Dist",  5.0f),
                knob ("tone",  "Tone",  5.0f),
                knob ("level", "Level", 5.0f));

    layout.add (std::make_unique<AudioParameterChoice> (pid ("turbo"), "Turbo", StringArray { "I", "II" }, 0));

    auto db = [] (const char* id, const char* name)
    {
        return std::make_unique<AudioParameterFloat> (
            pid (id), name, NormalisableRange<float> (-24.0f, 24.0f, 0.1f), 0.0f,
            AudioParameterFloatAttributes().withLabel ("dB"));
    };
    layout.add (db ("input", "Input Trim"), db ("output", "Output Trim"));

    layout.add (std::make_unique<AudioParameterChoice> (
        pid ("quality"), "Oversampling", StringArray { "2x", "4x", "8x" }, 1,
        AudioParameterChoiceAttributes().withAutomatable (false)));

    return layout;
}

//==============================================================================
bool TwinModeDistProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
    const auto out = layouts.getMainOutputChannelSet();
    if (out != juce::AudioChannelSet::mono() && out != juce::AudioChannelSet::stereo())
        return false;
    return layouts.getMainInputChannelSet() == out;
}

void TwinModeDistProcessor::prepareToPlay (double sampleRate, int samplesPerBlock)
{
    hostSampleRate = sampleRate;
    maxBlockSize   = juce::jmax (1, samplesPerBlock);
    const auto numChannels = (size_t) juce::jlimit (1, kMaxChannels, getTotalNumOutputChannels());

    for (int q = 0; q < kNumQualities; ++q)
    {
        oversamplers[(size_t) q] = std::make_unique<juce::dsp::Oversampling<float>> (
            numChannels, (size_t) (q + 1),
            juce::dsp::Oversampling<float>::filterHalfBandPolyphaseIIR,
            true,    // max quality filters
            true);   // integer latency
        oversamplers[(size_t) q]->initProcessing ((size_t) samplesPerBlock);
        latencies[(size_t) q] = (int) std::lround (oversamplers[(size_t) q]->getLatencyInSamples());
    }

    activeQuality = -1;
    activateQuality ((int) quality->load());
    wantedLatency.store (latencies[(size_t) activeQuality]);
    setLatencySamples (wantedLatency.load());
}

void TwinModeDistProcessor::activateQuality (int index)
{
    activeQuality = juce::jlimit (0, kNumQualities - 1, index);
    const double rate = hostSampleRate * (double) (2 << activeQuality);   // 2x, 4x, 8x

    for (auto& m : models)
    {
        m.setDist  (dist->load()  / 10.0f);
        m.setTone  (tone->load()  / 10.0f);
        m.setLevel (level->load() / 10.0f);
        m.setTurbo (turbo->load() > 0.5f ? 2 : 1);
        m.setInputTrimDb  (inTrim->load());
        m.setOutputTrimDb (outTrim->load());
        m.prepare (rate);   // pure arithmetic, no allocation
    }
    oversamplers[(size_t) activeQuality]->reset();
}

void TwinModeDistProcessor::timerCallback()
{
    const int wanted = wantedLatency.load();
    if (wanted != getLatencySamples())
        setLatencySamples (wanted);
}

//==============================================================================
void TwinModeDistProcessor::processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer&)
{
    juce::ScopedNoDenormals noDenormals;

    const int numSamples  = buffer.getNumSamples();
    const int numChannels = juce::jmin (getTotalNumOutputChannels(), kMaxChannels);

    for (int ch = getTotalNumInputChannels(); ch < getTotalNumOutputChannels(); ++ch)
        buffer.clear (ch, 0, numSamples);

    const int q = juce::jlimit (0, kNumQualities - 1, (int) quality->load());
    if (q != activeQuality)
    {
        activateQuality (q);
        wantedLatency.store (latencies[(size_t) q]);   // picked up by timerCallback()
    }

    for (int ch = 0; ch < numChannels; ++ch)
    {
        auto& m = models[(size_t) ch];
        m.setDist  (dist->load()  / 10.0f);
        m.setTone  (tone->load()  / 10.0f);
        m.setLevel (level->load() / 10.0f);
        m.setTurbo (turbo->load() > 0.5f ? 2 : 1);
        m.setInputTrimDb  (inTrim->load());
        m.setOutputTrimDb (outTrim->load());
    }

    auto& os = *oversamplers[(size_t) activeQuality];

    // Some hosts occasionally send more samples than announced: work in prepared-size chunks.
    for (int start = 0; start < numSamples; start += maxBlockSize)
    {
        const int len = juce::jmin (maxBlockSize, numSamples - start);
        juce::dsp::AudioBlock<float> block (buffer.getArrayOfWritePointers(), (size_t) numChannels,
                                            (size_t) start, (size_t) len);
        auto up = os.processSamplesUp (block);

        for (size_t ch = 0; ch < up.getNumChannels(); ++ch)
            models[ch].process (up.getChannelPointer (ch), (int) up.getNumSamples());

        os.processSamplesDown (block);
    }
}

//==============================================================================
juce::AudioProcessorEditor* TwinModeDistProcessor::createEditor()
{
    return new juce::GenericAudioProcessorEditor (*this);
}

void TwinModeDistProcessor::getStateInformation (juce::MemoryBlock& destData)
{
    if (auto xml = apvts.copyState().createXml())
        copyXmlToBinary (*xml, destData);
}

void TwinModeDistProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    if (auto xml = getXmlFromBinary (data, sizeInBytes))
        if (xml->hasTagName (apvts.state.getType()))
            apvts.replaceState (juce::ValueTree::fromXml (*xml));
}

//==============================================================================
juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new TwinModeDistProcessor();
}
