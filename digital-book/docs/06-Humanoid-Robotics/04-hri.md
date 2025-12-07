---
id: humanoid-hri
title: HRI
sidebar_label: HRI
sidebar_position: 4
---

## Understanding Human-Robot Interaction (HRI) {#ch6-sec4-understanding-hri}

**Human-Robot Interaction (HRI)** focuses on enabling natural, intuitive, and safe communication between humans and humanoid robots. HRI combines elements of robotics, computer vision, natural language processing, and cognitive science to create robots that can understand human intentions, emotions, and social cues.

### Key HRI Concepts: {#ch6-sec4-hri-concepts}

1.  **Social Cues**: Understanding gestures, facial expressions, and body language
2.  **Proxemics**: Managing interpersonal distance and personal space
3.  **Turn-taking**: Natural conversation flow and interaction timing
4.  **Intent Recognition**: Inferring human goals and intentions
5.  **Emotion Detection**: Recognizing and responding to human emotions
6.  **Safety Protocols**: Ensuring physical and psychological safety

### HRI System Architecture: {#ch6-sec4-hri-architecture}

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from enum import Enum
import time

class State(Enum): IDLE=0; LISTENING=1; PROCESSING=2; RESPONDING=3

class HRI(Node):
    def __init__(self):
        super().__init__('hri')
        self.state = State.IDLE
        self.speech = self.gesture = self.intent = None
        self.face = True
        self.sp_pub = self.create_publisher(String,'/speech/output',10)
        self.g_pub = self.create_publisher(String,'/gesture/command',10)
        self.led_pub = self.create_publisher(String,'/led/expression',10)
        self.create_timer(0.1,self.loop)
        self.create_subscription(String,'/speech/input',self.cb_speech,10)
        self.create_subscription(Image,'/camera/gestures',self.cb_gesture,10)

    def cb_speech(self,msg): self.speech = msg.data
    def cb_gesture(self,msg): self.gesture = "wave"

    def loop(self):
        if self.state==State.IDLE and self.face:
            self.say("Hello!"); self.state=State.LISTENING
        elif self.state==State.LISTENING and (self.speech or self.gesture):
            self.state=State.PROCESSING
        elif self.state==State.PROCESSING:
            self.intent = self.speech_intent(self.speech) if self.speech else self.gesture_intent(self.gesture)
            self.state=State.RESPONDING
        elif self.state==State.RESPONDING:
            self.respond(); self.speech=self.gesture=self.intent=None; self.state=State.IDLE

    def speech_intent(self,text):
        text=text.lower()
        if any(w in text for w in ['hello','hi']): return 'greeting'
        if any(w in text for w in ['help']): return 'assistance'
        if any(w in text for w in ['follow','come']): return 'follow'
        if any(w in text for w in ['stop']): return 'stop'
        return 'unknown'

    def gesture_intent(self,g): return {'wave':'greeting','beckon':'follow','stop_palm':'stop'}.get(g,'unknown')

    def respond(self):
        r={'greeting':("Hi!",'wave','happy'),'assistance':("How can I help?",'open','helpful'),
           'follow':("I will follow",'nod','attentive'),'stop':("Stopping",'stop','neutral'),
           'unknown':("I didn't understand",'head_tilt','confused')}
        s,g,e = r.get(self.intent,r['unknown'])
        self.say(s); self.perform(g); self.led(e)

    def say(self,text): m=String();m.data=text;self.sp_pub.publish(m)
    def perform(self,g): m=String();m.data=g;self.g_pub.publish(m)
    def led(self,e): m=String();m.data=e;self.led_pub.publish(m)

def main():
    rclpy.init(); h=HRI()
    try: rclpy.spin(h)
    except KeyboardInterrupt: pass
    finally: h.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
```

### Proxemics Management: {#ch6-sec4-proxemics}

```python
class ProxemicsManager:
    """Manage interpersonal distance based on social norms."""
    
    def __init__(self):
        # Social distance zones (in meters)
        self.zones = {
            'intimate': (0.0, 0.45),   # Very close
            'personal': (0.45, 1.2),   # Normal conversation
            'social': (1.2, 3.6),      # Social interactions
            'public': (3.6, 7.6)       # Public speaking
        }
    
    def adjust_distance(self, human_position, current_zone='social'):
        """Adjust robot position to maintain appropriate distance."""
        target_distance = np.mean(self.zones[current_zone])
        
        current_distance = np.linalg.norm(human_position)
        adjustment = target_distance - current_distance
        
        # Move slowly to avoid startling
        if abs(adjustment) > 0.1:
            return adjustment * 0.5  # Slow adjustment
        return 0.0
```

### Safety Systems: {#ch6-sec4-safety}

```python
class SafetyMonitor:
    """Monitor and ensure safe HRI."""
    
    def __init__(self):
        self.safety_zones = {
            'critical': 0.3,   # No-go zone
            'warning': 0.6,    # Slow movement
            'normal': 1.0      # Full speed
        }
    
    def check_safety(self, human_position, robot_velocity):
        """Check if interaction is safe."""
        distance = np.linalg.norm(human_position)
        
        if distance < self.safety_zones['critical']:
            return 'emergency_stop'
        elif distance < self.safety_zones['warning']:
            return 'reduce_speed'
        else:
            return 'normal'
```

### HRI Modalities: {#ch6-sec4-modalities}

| Modality | Input | Output | Technologies |
|----------|-------|--------|--------------|
| **Speech** | Microphone | Speaker | ASR, TTS, NLP |
| **Gestures** | Cameras | Robot Arms | CV, Pose Estimation |
| **Gaze** | Eye tracking | LED display | Eye tracking, Animation |
| **Touch** | Force sensors | Haptic feedback | Pressure sensing, Vibration |
| **Facial** | Camera | Display/face | Emotion recognition, Animation |


### Common HRI Patterns: {#ch6-sec4-patterns}

*   **Command-Response**: Human commands, robot executes
*   **Collaborative**: Human and robot work together
*   **Supervisory**: Human monitors, robot operates autonomously
*   **Social**: Natural conversation and interaction
*   **Teaching**: Human demonstrates, robot learns
*   **Assistive**: Robot helps with tasks

### Challenges: {#ch6-sec4-challenges}

*   **Ambiguity Resolution**: Understanding unclear human commands
*   **Context Awareness**: Understanding situation and environment
*   **Natural Language**: Handling colloquial speech and slang
*   **Cultural Differences**: Adapting to different social norms
*   **Trust Building**: Establishing and maintaining human trust
*   **Long-term Interaction**: Maintaining engagement over time

### References and Further Reading: {#ch6-sec4-references}

*   **Book**: *[Human-Robot Interaction: An Introduction](https://www.cambridge.org/core/books/humanrobot-interaction/)* by Christoph Bartneck et al.
*   **Standard**: [ISO/TS 15066:2016](https://www.iso.org/standard/62996.html) - Collaborative robots safety
*   **Conference**: [ACM/IEEE HRI Conference](https://humanrobotinteraction.org/) proceedings
*   **Toolkit**: [ROS HRI Packages](http://wiki.ros.org/hri) for ROS-based HRI
*   **Library**: [OpenFace](https://github.com/TadasBaltrusaitis/OpenFace) for facial analysis
*   **Dataset**: [MULTISIMO](https://multisimo.eu/) multimodal HRI dataset
