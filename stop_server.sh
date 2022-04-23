echo "Stopping the docker container..."
eval 'docker stop rosns3-server'
sleep 2
eval 'docker rm rosns3-server'
echo "Stopped and removed the docker container."
