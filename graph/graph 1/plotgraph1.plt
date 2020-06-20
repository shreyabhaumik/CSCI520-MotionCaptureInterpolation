set terminal png size 1280,720
set output "Graph#1.png"
set title "Linear Euler vs Bezier Euler vs Input"
set xlabel "Frame Number"
set ylabel "Angle (degrees)"
plot "Graph1_LEvsBEvsIP_600-800f_N20.txt" using 1:2 with lines title "Linear Euler" lw 2, "Graph1_LEvsBEvsIP_600-800f_N20.txt" using 1:3 with lines title "Bezier Euler" lw 2, "Graph1_LEvsBEvsIP_600-800f_N20.txt" using 1:4 with lines title "Input" lw 2