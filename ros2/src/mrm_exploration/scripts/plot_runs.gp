# Plot explored area over time for one or more runs on the same graph.
#
#   sudo apt install gnuplot
#   cd ~/mrm_ws/exploration_runs
#   gnuplot -c <path-to>/plot_runs.gp one_robot/metrics.csv two_uncoordinated/metrics.csv two_coordinated/metrics.csv
#
# Output: exploration_comparison.png in the current folder.
# Each line is labelled with its run folder name.
set datafile separator ","
set terminal pngcairo size 1000,600 font "sans,12"
set output "exploration_comparison.png"
set title "Multi-robot frontier exploration: explored area over time"
set xlabel "time (s)"
set ylabel "explored area (m^2)"
set key bottom right
set grid
label(f) = system("basename $(dirname " . f . ")")
plot for [i=1:ARGC] value("ARG".i) every ::1 using 1:2 with lines linewidth 2 title label(value("ARG".i)) noenhanced
