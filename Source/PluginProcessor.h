#pragma once

#include <juce_audio_processors/juce_audio_processors.h>
#include <juce_dsp/juce_dsp.h>

#include "DS2Model.h"

#include <array>
#include <atomic>
#include <memory>

class TwinModeDistProcessor final : public juce::AudioProcessor,
                                    private juce::Timer
{
public:
    TwinModeDistProcessor();
    ~TwinModeDistProcessor() override;

    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override {}
    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;

    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;
    using AudioProcessor::processBlock;

    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override { return true; }

    const juce::String getName() const override { return JucePlugin_Name; }
    bool acceptsMidi() const override { return false; }
    bool producesMidi() const override { return false; }
    bool isMidiEffect() const override { return false; }
    double getTailLengthSeconds() const override { return 0.0; }

    int getNumPrograms() override { return 1; }
    int getCurrentProgram() override { return 0; }
    void setCurrentProgram (int) override {}
    const juce::String getProgramName (int) override { return {}; }
    void changeProgramName (int, const juce::String&) override {}

    void getStateInformation (juce::MemoryBlock& destData) override;
    void setStateInformation (const void* data, int sizeInBytes) override;

    static juce::AudioProcessorValueTreeState::ParameterLayout createParameterLayout();
    juce::AudioProcessorValueTreeState apvts;

private:
    void timerCallback() override;       // reports latency changes from the message thread
    void activateQuality (int index);    // audio thread; no allocation

    static constexpr int kMaxChannels = 2;
    static constexpr int kNumQualities = 3;   // 2x, 4x, 8x

    std::array<std::unique_ptr<juce::dsp::Oversampling<float>>, kNumQualities> oversamplers;
    std::array<ds2::DS2Model, kMaxChannels> models;
    std::array<int, kNumQualities> latencies {};

    double hostSampleRate = 44100.0;
    int maxBlockSize = 512;
    int activeQuality = -1;
    std::atomic<int> wantedLatency { 0 };

    std::atomic<float>* dist    = nullptr;
    std::atomic<float>* tone    = nullptr;
    std::atomic<float>* level   = nullptr;
    std::atomic<float>* turbo   = nullptr;
    std::atomic<float>* inTrim  = nullptr;
    std::atomic<float>* outTrim = nullptr;
    std::atomic<float>* quality = nullptr;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (TwinModeDistProcessor)
};
