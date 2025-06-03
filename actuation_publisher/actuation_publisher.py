#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge
import cv2
from openai import OpenAI
import time
import base64

client = OpenAI()


class TrafficLightImageRelay(Node):

    def __init__(self):
        super().__init__('traffic_light_image_relay')

        # Best Effort QoS
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/sensing/camera/traffic_light/image_raw',
            self.listener_callback,
            qos_profile
        )

        # Publisher (uses same QoS)
        self.publisher = self.create_publisher(
            Image,
            '/processed/traffic_light/image_raw',
            qos_profile
        )

        self.bridge = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info('Received image; sending to GPT and annotating')

        try:
            # Convert ROS Image → OpenCV BGR image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Encode BGR → JPEG in memory
            ret, jpeg_buf = cv2.imencode('.jpg', cv_image)
            if not ret:
                self.get_logger().warn("JPEG encoding failed")
                return

            # Base64-encode JPEG bytes
            jpeg_bytes = jpeg_buf.tobytes()
            base64_image = base64.b64encode(jpeg_bytes).decode('utf-8')

            # Call GPT with image
            response = client.responses.create(
                model="gpt-4.1-mini",
                input=[{
                    "role": "user",
                    "content": [
                        {"type": "input_text",
                         "text": "what's in this image, describe in great detail"},
                        {
                            "type": "input_image",
                            # Pass as a data URI
                            "image_url": f"data:image/jpeg;base64,{base64_image}"
                        },
                    ],
                }],
            )

            # Extract GPT’s textual response
            text_to_overlay = response.output_text.strip()
            print(text_to_overlay)
            self.get_logger().info(f"GPT response: {text_to_overlay}")

            # --- Overlay text onto cv_image ---
            # Choose font and scale
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1
            font_thickness = 2

            # Split into lines if too long
            max_width = cv_image.shape[1] - 20  # leave 10px margin on each side
            lines = []
            for line in text_to_overlay.split('\n'):
                # Simple word-wrap: break each line into chunks that fit
                words = line.split(' ')
                current = ""
                for w in words:
                    test = current + (" " if current else "") + w
                    ((text_w, _), _) = cv2.getTextSize(test, font, font_scale, font_thickness)
                    if text_w > max_width:
                        lines.append(current)
                        current = w
                    else:
                        current = test
                if current:
                    lines.append(current)

            # Background rectangle settings
            margin = 5
            line_height = cv2.getTextSize("Tg", font, font_scale, font_thickness)[0][1] + 5
            overlay_height = line_height * len(lines) + 2 * margin
            overlay_width = max(
                cv2.getTextSize(ln, font, font_scale, font_thickness)[0][0] for ln in lines
            ) + 2 * margin

            # Draw semi-transparent background rectangle in top-left corner
            overlay = cv_image.copy()
            cv2.rectangle(
                overlay,
                (0, 0),
                (overlay_width, overlay_height),
                (0, 0, 0),  # black background
                -1
            )
            alpha = 0.5  # transparency factor
            cv_image = cv2.addWeighted(overlay, alpha, cv_image, 1 - alpha, 0)

            # Write each line of text
            y0 = margin + line_height - 5
            for i, ln in enumerate(lines):
                y = y0 + i * line_height
                cv2.putText(
                    cv_image,
                    ln,
                    (margin, y),
                    font,
                    font_scale,
                    (255, 255, 255),  # white text
                    font_thickness,
                    lineType=cv2.LINE_AA
                )

            # Allow some time (optional)
            time.sleep(1)

            # Convert annotated OpenCV image → ROS Image msg
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

            # Preserve header (timestamp/frame_id) if you want:
            annotated_msg.header = msg.header

            # Publish the annotated image
            self.publisher.publish(annotated_msg)
            self.get_logger().info('Published annotated image')

        except Exception as e:
            self.get_logger().warn(f"Error in processing: {e}")

    def destroy_node(self):
        # Clean up any OpenCV windows if you used imshow (not used here)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightImageRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
