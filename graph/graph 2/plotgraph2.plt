set terminal png size 1280,720
set output "Graph#2.png"
set title "Linear Quaternion vs Bezier Quaternion vs Input"
set xlabel "Frame Number"
set ylabel "Angle (degrees)"
plot "Graph2_LQvsBQvsIP_600-800f_N20.txt" using 1:2 with lines title "Linear Quaternion" lw 2, "Graph2_LQvsBQvsIP_600-800f_N20.txt" using 1:3 with lines title "Bezier Quaternion" lw 2, "Graph2_LQvsBQvsIP_600-800f_N20.txt" using 1:4 with lines title "Input" lw 2