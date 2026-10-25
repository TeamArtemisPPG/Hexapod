#!/usr/bin/env python3
"""
voice_bridge_node.py — Voice Input (STT) and Voice Output (TTS) for Zoe.

Captures microphone audio via ALSA arecord, transcribes using SpeechRecognition,
publishes to /brain/query, listens to /brain/response, and speaks responses aloud
via system speech-dispatcher (spd-say).

Usage:
  ros2 run rover voice_bridge
"""

import os
import sys
import time
import threading
import subprocess
import tempfile

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Ensure SpeechRecognition from user site-packages is importable
USER_SITE = '/home/vishal/.local/lib/python3.12/site-packages'
if os.path.isdir(USER_SITE) and USER_SITE not in sys.path:
    sys.path.insert(0, USER_SITE)

try:
    import speech_recognition as sr
    HAVE_SR = True
except ImportError:
    HAVE_SR = False


class VoiceBridgeNode(Node):

    def __init__(self):
        super().__init__('voice_bridge_node')

        self.declare_parameter('device', 'default')
        self.declare_parameter('record_duration_sec', 8.0)
        self.declare_parameter('enable_tts', True)
        self.declare_parameter('auto_listen', True)

        self._device = self.get_parameter('device').value
        self._record_sec = float(self.get_parameter('record_duration_sec').value)
        self._enable_tts = bool(self.get_parameter('enable_tts').value)
        self._auto_listen = bool(self.get_parameter('auto_listen').value)

        # Publisher to Brain Query
        self._query_pub = self.create_publisher(String, '/brain/query', 10)

        # Subscriber to Brain Response
        self._response_sub = self.create_subscription(
            String, '/brain/response', self._on_brain_response, 10
        )

        self._running = True
        self._recognizer = sr.Recognizer() if HAVE_SR else None

        self.get_logger().info('🎙️ VoiceBridgeNode ready!')
        self.get_logger().info('Speak into your microphone starting with: "Hey Zoe ..."')

        # Start listening background thread
        if self._auto_listen and HAVE_SR:
            self._listen_thread = threading.Thread(target=self._listen_loop, daemon=True)
            self._listen_thread.start()
        elif not HAVE_SR:
            self.get_logger().error(
                'SpeechRecognition package not available. Voice input disabled.'
            )

    def _on_brain_response(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f'🔊 Zoe says: "{text}"')

        # Speak via system TTS if enabled
        if self._enable_tts:
            threading.Thread(target=self._speak, args=(text,), daemon=True).start()

    def _speak(self, text: str):
        # Filter out UI emojis for cleaner pronunciation
        clean_text = text.replace('⚠️', 'Warning: ').replace('❌', 'Error: ').replace('✅', '').replace('👂', '').replace('👋', '').replace('🗺️', '')
        try:
            subprocess.run(
                ['spd-say', '-t', 'female2', '-r', '10', clean_text],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10
            )
        except Exception as e:
            self.get_logger().warn(f'TTS speak error: {e}')

    def _listen_loop(self):
        """Continuous audio capture and speech-to-text loop."""
        time.sleep(2.0)  # Wait for node initialization
        while rclpy.ok() and self._running:
            wav_path = None
            try:
                with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tf:
                    wav_path = tf.name

                # Record audio snippet via arecord
                # 16kHz, 16-bit Mono is optimal for speech recognition
                cmd = [
                    'arecord',
                    '-q',
                    '-d', str(int(self._record_sec)),
                    '-f', 'S16_LE',
                    '-r', '16000',
                    '-c', '1',
                    wav_path
                ]

                proc = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=self._record_sec + 2)
                if proc.returncode != 0 or not os.path.exists(wav_path) or os.path.getsize(wav_path) < 1000:
                    time.sleep(0.5)
                    continue

                # Transcribe with SpeechRecognition
                with sr.AudioFile(wav_path) as source:
                    self._recognizer.adjust_for_ambient_noise(source, duration=0.3)
                    audio_data = self._recognizer.record(source)

                try:
                    text = self._recognizer.recognize_google(audio_data)
                except sr.UnknownValueError:
                    # Inaudible / silence — quietly continue
                    text = None
                except sr.RequestError as e:
                    self.get_logger().warn(f'Speech recognition service error: {e}')
                    text = None

                if text:
                    text = text.strip()
                    self.get_logger().info(f'🎙️ [Heard] "{text}"')

                    # Check if wake word Zoe or phonetic variant is present
                    lower_text = text.lower()
                    if any(w in lower_text for w in ['zoe', 'zoey', 'joey', 'chloe', 'joy']):
                        self.get_logger().info(f'🚀 [Triggered Zoe] Sending to /brain/query: "{text}"')
                        self._query_pub.publish(String(data=text))
                        # Give brain time to process before next recording
                        time.sleep(3.0)
                    else:
                        self.get_logger().debug(f'Ignored speech without Zoe: "{text}"')

            except Exception as e:
                self.get_logger().warn(f'Listen loop warning: {e}')
                time.sleep(1.0)
            finally:
                if wav_path and os.path.exists(wav_path):
                    try:
                        os.remove(wav_path)
                    except OSError:
                        pass

    def stop(self):
        self._running = False


def main(args=None):
    rclpy.init(args=args)
    node = VoiceBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
