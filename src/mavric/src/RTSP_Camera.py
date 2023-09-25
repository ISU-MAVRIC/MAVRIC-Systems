import cv2
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

# Initialize GStreamer
Gst.init(None)

# Create a VideoCapture object for the webcam or video source
cap = cv2.VideoCapture(0)  # Change to the appropriate device index or video file path

# Check if the VideoCapture object opened successfully
if not cap.isOpened():
    print("Error: Could not open video source.")
    exit()

# Define the RTSP server URL
rtsp_url = "rtsp://localhost:8554/test"  # Change the URL as needed

# Create a GStreamer pipeline for encoding and streaming
pipeline = Gst.Pipeline()

# Create a Bin element to group multiple elements
bin = Gst.Bin.new("my-bin")

# Create elements for the pipeline
source = Gst.ElementFactory.make("appsrc", "source")
encoder = Gst.ElementFactory.make("x264enc", "encoder")
rtph264pay = Gst.ElementFactory.make("rtph264pay", "rtph264pay")
udpsink = Gst.ElementFactory.make("udpsink", "udpsink")

# Set the properties for the elements
source.set_property("is-live", True)
source.set_property("block", True)
source.set_property("format", Gst.Format.TIME)

udpsink.set_property("host", "127.0.0.1")  # Change to your RTSP server's IP address
udpsink.set_property("port", 8554)

# Add elements to the bin and bin to the pipeline
bin.add(source)
bin.add(encoder)
bin.add(rtph264pay)
bin.add(udpsink)

# Link the elements together
source.link(encoder)
encoder.link(rtph264pay)
rtph264pay.link(udpsink)

# Set the bin as the pipeline's source
pipeline.add(bin)

# Set the pipeline state to playing
pipeline.set_state(Gst.State.PLAYING)

# Start capturing and streaming frames
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert the frame to a GStreamer buffer
    ret, buffer = cv2.imencode('.jpeg', frame)
    if ret:
        gst_buffer = Gst.Buffer.new_allocate(None, len(buffer), None)
        gst_buffer.fill(0, buffer.tobytes())

        # Push the buffer into the GStreamer pipeline
        source.emit("push-buffer", gst_buffer)

# Release the VideoCapture and pipeline resources
cap.release()
pipeline.set_state(Gst.State.NULL)