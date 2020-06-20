set terminal png size 1280,720
set output "Graph#5.png"
set title "Linear SLERP Quaternion vs Linear SLERP and LERP Quaternion vs Input"
set xlabel "Frame Number"
set ylabel "Angle (degrees)"
plot "Graph5_LSQvsLSLQvsIP_200-500f_N20.txt" using 1:2 with lines title "Linear SLERP Quaternion" lw 2, "Graph5_LSQvsLSLQvsIP_200-500f_N20.txt" using 1:3 with lines title "Linear SLERP and LERP Quaternion" lw 2, "Graph5_LSQvsLSLQvsIP_200-500f_N20.txt" using 1:4 with lines title "Input" lw 2