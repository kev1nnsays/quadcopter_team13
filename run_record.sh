# set environment variables for lcm-spy
. ./setenv.sh

# start lcm-spy
lcm-spy &

# plot the lcm messages
gnuplot -persist -e "config='loop.plt'" loop.plt &
