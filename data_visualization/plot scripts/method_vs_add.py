import matplotlib.pyplot as plt
import pandas as pd
import matplotlib as mpl

# Load the data from the CSV file
df = pd.read_csv('performance_metrics.csv')

# Extract the x and y values from the data
# methods = df['Method']
# x = df['Inference time']
# y1 = df['accuracy_1']
# y2 = df['accuracy_2']

configurations = df['Method Configuration'].unique()
markers = ['*', '^', 'H', 'v', '^', '<', '>', 'p', 'P', '*', 'h', 'H', 'X', 'D']
colors = plt.cm.Set1.colors

# Create a new figure and set the size of the plot
fig = plt.figure(figsize=(3.5, 2.5))
# fig = plt.figure(figsize=(8, 6))

for i, configuration in enumerate(configurations):
    data = df[df['Method Configuration'] == configuration]
    x = data['Inference Time']
    y = data['Average Recall']
    plt.plot(x, y, '-o', color=colors[i], marker=markers[i], label=configuration, markersize=10)

# Add a legend to the plot
plt.legend(loc='lower right')

# Set the font size and style
mpl.rcParams.update({
    'font.size': 9,
    'font.family': 'serif',
    'font.serif': ['Times'],
    'text.usetex': True
})

# Plot the line graph
# plt.plot(x, y, '-o')

# Set the title and axis labels
plt.title('Performance Metrics of Different Method Configurations')
plt.xlabel('Method Configuration')
plt.ylabel('Performance Metrics')

# Set the x-axis tick labels to display at a 45-degree angle
plt.xticks(rotation=45)

# Save the figure as a PDF file
fig.savefig('performance_metrics.pdf', dpi=300, bbox_inches='tight')

# Show the plot
plt.show()
