set terminal png size 1280,720
set output "Graph#3.png"
set title "Linear Euler vs Linear Quaternion vs Input"
set xlabel "Frame Number"
set ylabel "Angle (degrees)"
plot "Graph3_LEvsLQvsIP_200-500f_N20.txt" using 1:2 with lines title "Linear Euler" lw 2, "Graph3_LEvsLQvsIP_200-500f_N20.txt" using 1:3 with lines title "Linear Quaternion" lw 2, "Graph3_LEvsLQvsIP_200-500f_N20.txt" using 1:4 with lines title "Input" lw 2