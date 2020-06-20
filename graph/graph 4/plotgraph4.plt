set terminal png size 1280,720
set output "Graph#4.png"
set title "Bezier Euler vs Bezier Quaternion vs Input"
set xlabel "Frame Number"
set ylabel "Angle (degrees)"
plot "Graph4_BEvsBQvsIP_200-500f_N20.txt" using 1:2 with lines title "Bezier Euler" lw 2, "Graph4_BEvsBQvsIP_200-500f_N20.txt" using 1:3 with lines title "Bezier Quaternion" lw 2, "Graph4_BEvsBQvsIP_200-500f_N20.txt" using 1:4 with lines title "Input" lw 2