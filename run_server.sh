# Enable debug logs for ROSNS3 Server
ENABLE_LOGGING=0

# Friis loss model parameters
path_loss_exponent=2.4
reference_loss=46;
transmission_power=16.0206;
fading_mean=0;
fading_var=32;

# WAF command to run rosns3 server
# CMD_WAF="./waf --run 'rosns3-example --exp=$path_loss_exponent --ref_loss=$reference_loss --tx_power=$transmission_power --mean=$fading_mean --var=$fading_var'"

# Docker command to run the image
CMD_RUN_IMAGE="docker run -t -d --privileged -p 28500:28500 --name rosns3-server malinthaf/rosns3-server"
eval $CMD_RUN_IMAGE
sleep 2

# Docker command to run the server without logging
if [ $ENABLE_LOGGING -eq 0 ] ;
then
    echo "Running ROSNS3 without logging"
    eval "docker exec -i  rosns3-server bash -c 'cd /usr/ns3/ns-3.30 && ./waf --run 'rosns3-example --exp=$path_loss_exponent --ref_loss=$reference_loss --tx_power=$transmission_power --mean=$fading_mean --var=$fading_var''"
fi

# Docker command to run the server with logging
if [ $ENABLE_LOGGING -eq 1 ] ;
then
    eval "docker exec -i  rosns3-server bash -c 'cd /usr/ns3/ns-3.30 && NS_LOG='ROSNS3Server:ROSNS3Example:ROSNS3Model' ./waf --run 'rosns3-example --exp=$path_loss_exponent --ref_loss=$reference_loss --tx_power=$transmission_power --mean=$fading_mean --var=$fading_var''"
fi
