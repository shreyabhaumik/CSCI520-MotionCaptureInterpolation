set terminal png size 1280,720
set output "Graph#6.png"
set title "Bezier SLERP Quaternion vs Bezier SLERP and LERP Quaternion vs Input"
set xlabel "Frame Number"
set ylabel "Angle (degrees)"
plot "Graph6_BSQvsBSLQvsIP_200-500f_N20.txt" using 1:2 with lines title "Bezier SLERP Quaternion" lw 2, "Graph6_BSQvsBSLQvsIP_200-500f_N20.txt" using 1:3 with lines title "Bezier SLERP and LERP Quaternion" lw 2, "Graph6_BSQvsBSLQvsIP_200-500f_N20.txt" using 1:4 with lines title "Input" lw 2