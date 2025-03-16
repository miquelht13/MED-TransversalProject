import asyncio
import websockets
import cv2
import base64

video_clients = set()
command_clients = set()

async def process_video(websocket, path=None):
    video_clients.add(websocket)
    try:
        print("Video client connected")
        cap = cv2.VideoCapture(0)
        frame_count = 0

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            frame_count += 1
            print(f"Processing frame {frame_count}...")

            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, frame_data = cv2.imencode('.jpg', gray_frame)
            base64_frame = base64.b64encode(frame_data).decode("utf-8")

            await websocket.send(base64_frame)
            await asyncio.sleep(0.1)

        cap.release()
        print("Video processing complete. Closing connection.")
        await websocket.close()

    except websockets.exceptions.ConnectionClosedError:
        print("Video client disconnected. Waiting for a new connection...")
    except Exception as e:
        print(f"Error on the video server: {str(e)}")
    finally:
        video_clients.remove(websocket)

async def process_commands(websocket, path=None):
    command_clients.add(websocket)
    try:
        print("Command client connected")
        while True:
            command = await websocket.recv()
            print(f"Received command: {command}")

    except websockets.exceptions.ConnectionClosedError:
        print("Command client disconnected. Waiting for a new connection...")
    except Exception as e:
        print(f"Error on the command server: {str(e)}")
    finally:
        command_clients.remove(websocket)

async def main():
    video_server = websockets.serve(process_video, "localhost", 8765)
    command_server = websockets.serve(process_commands, "localhost", 8766)
    await asyncio.gather(video_server, command_server)
    print("WebSocket servers started.")
    await asyncio.Future()

if __name__ == "__main__":
    import sys

    if sys.platform.startswith("win"):
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

    asyncio.run(main())