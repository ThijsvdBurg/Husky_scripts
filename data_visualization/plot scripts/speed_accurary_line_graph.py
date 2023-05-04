import matplotlib.pyplot as plt
import pandas as pd
import matplotlib as mpl

# Load the data from the CSV file
df = pd.read_csv('/home/pmvanderburg/noetic-husky/bop_toolkit/existing_file.csv')

configurations = df['Method Configuration'].unique()
# pnp_cfs = df['PnP Iterations']
markers = ['*', '^', 'H', 'v', '^', '<', '>', 'p', 'P', '*', 'h', 'H', 'X', 'D']
colors = plt.cm.Set1.colors

# Create a new figure and set the size of the plot
# fig = plt.figure(figsize=(3.5, 2.5))
fig = plt.figure(figsize=(8, 6))

idx=0
for i, configuration in enumerate(configurations):
    datapoints = df[df['Method Configuration'] == configuration]
    # num_iters = df[df['PnP Iterations']]
    x = datapoints['Inference Time']
    y = datapoints['AUC of Average Recall']
    # label = data['PnP Iterations']
    # label = df[df['PnP Iterations'] == ]
    # label = pnp_cfs[i]
    plt.plot(x, y, '-o', color=colors[i], marker=markers[i], label=configuration, markersize=10)
    # Add labels to each data point
    labels = df['PnP Iterations']
    
    # print(datapoints, labels)
    for j, data_instance in enumerate(datapoints['PnP Iterations']):
        # if j>0:
        plt.text(x[idx], y[idx], labels[idx], fontsize=5, ha='center', va='bottom')
        idx+=1


    # for j, label in enumerate(labels):
        
# Add a legend to the plot
plt.legend(loc='lower right',prop={'size': 9})

# Set the font size and style
mpl.rcParams.update({
    'font.size': 10,
    'font.family': 'serif',
    'font.serif': ['Times'],
    'text.usetex': True
})

# Plot the line graph
# plt.plot(x, y, '-o')

# Set the title and axis labels
plt.title('Performance Metrics of Different Method Configurations')
plt.ylabel('AUC of Average recall of ADD')
plt.xlabel('Inference Time (ms)')

# Set the x-axis tick labels to display at a 45-degree angle
# plt.xticks(rotation=45)

# Save the figure as a PDF file
fig.savefig('performance_metrics.pdf', dpi=300, bbox_inches='tight')

# Show the plot
plt.show()
