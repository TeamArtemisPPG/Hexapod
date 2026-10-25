#!/usr/bin/env python3
"""
test_voice_talk.py — Interactive push-to-talk test utility for Zoe.

Press Enter, speak "Hey Zoe, go forward" into your mic,
and Zoe will transcribe your voice, send it to the brain, and speak back!
"""

import os
import sys
import subprocess
import tempfile
import time

USER_SITE = '/home/vishal/.local/lib/python3.12/site-packages'
if os.path.isdir(USER_SITE) and USER_SITE not in sys.path:
    sys.path.insert(0, USER_SITE)

import speech_recognition as sr
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    print("=" * 60)
    print("🎙️  ZOE PUSH-TO-TALK VOICE UTILITY")
    print("=" * 60)
    print("Speak clearly starting with: 'Hey Zoe ...'")
    print("Examples:")
    print("  • 'Hey Zoe, go forward'")
    print("  • 'Hey Zoe, rotate 90 degrees'")
    print("  • 'Hey Zoe, check your sensors'")
    print("  • 'Hey Zoe, roam around the area that is not mapped and map the entire place'")
    print("=" * 60)

    rclpy.init()
    node = Node('interactive_voice_tester')
    pub = node.create_publisher(String, '/brain/query', 10)

    def on_resp(msg):
        print(f"\n📢 [Zoe's Brain Response]: {msg.data}")
        # Speak response aloud
        subprocess.run(
            ['spd-say', '-t', 'female2', '-r', '10', msg.data.replace('⚠️', '').replace('✅', '').replace('👂', '').replace('👋', '').replace('🗺️', '')],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

    sub = node.create_subscription(String, '/brain/response', on_resp, 10)
    recognizer = sr.Recognizer()

    try:
        while True:
            input("\n👉 Press [ENTER] to START speaking... (Ctrl+C to quit)")
            print("\n" + "🔴" * 20)
            print("🎙️  RECORDING... Speak clearly into your mic:")
            print("   'Hey Zoe, roam around the area that is not mapped and map the entire place'")
            print("👉 Press [ENTER] when you are DONE speaking...")
            print("🔴" * 20)

            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tf:
                wav_path = tf.name

            try:
                # Start arecord in background (safety timeout 20s)
                proc = subprocess.Popen(
                    ['arecord', '-q', '-d', '20', '-f', 'S16_LE', '-r', '16000', '-c', '1', wav_path]
                )

                # Wait for user to press Enter to stop
                input()

                # Terminate arecord cleanly
                if proc.poll() is None:
                    proc.terminate()
                    try:
                        proc.wait(timeout=1.0)
                    except subprocess.TimeoutExpired:
                        proc.kill()

                # Verify file has audio
                if not os.path.exists(wav_path) or os.path.getsize(wav_path) < 2000:
                    print("⚠️ Audio too short or empty. Please speak clearly into the microphone.")
                    continue

                print("⏳ Transcribing audio with Google Speech Recognition...")

                with sr.AudioFile(wav_path) as source:
                    recognizer.adjust_for_ambient_noise(source, duration=0.3)
                    audio = recognizer.record(source)

                text = recognizer.recognize_google(audio)
                print(f"\n🎙️  Transcribed text: \"{text}\"")

                # Check wake word
                lower_text = text.lower()
                if not any(w in lower_text for w in ['zoe', 'zoey', 'joey', 'chloe', 'joy']):
                    print("⚠️ Note: 'Hey Zoe' wake word was not heard. Prepending 'Hey Zoe, ' automatically...")
                    text = f"Hey Zoe, {text}"
                    print(f"🚀 Adjusted query: \"{text}\"")

                # Publish to Zoe's brain
                print("🚀 Publishing to /brain/query...")
                pub.publish(String(data=text))
                print("⏳ Waiting for Zoe's response (press Ctrl+C to return to mic prompt)...")

                # Spin to catch the response
                t0 = time.time()
                while time.time() - t0 < 15:
                    rclpy.spin_once(node, timeout_sec=0.2)

            except sr.UnknownValueError:
                print("❓ Could not understand audio. Please speak a little louder into the mic.")
            except sr.RequestError as e:
                print(f"❌ Speech recognition service error: {e}")
            except Exception as e:
                print(f"❌ Error: {e}")
            finally:
                if os.path.exists(wav_path):
                    try:
                        os.remove(wav_path)
                    except OSError:
                        pass

    except KeyboardInterrupt:
        print("\nExiting voice utility...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
