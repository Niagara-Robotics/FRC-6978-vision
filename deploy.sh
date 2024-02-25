hostname = "frc-6978-vision.local"

echo "[🔎] Looking for vision computer"
if ping -c 1 -W 1 frc-6978-vision.local; then
    echo "[✅] Found host"
else
    echo "[❗] Unable to find host"
    exit
fi

scp -r ./ robotics@frc-6978-vision.local:~/FRC-6978-vision
ssh robotics@frc-6978-vision.local bash -c 'cd FRC-6978-vision; ./payload.sh'