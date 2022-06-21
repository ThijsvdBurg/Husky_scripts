# Data Visu

Steps:

1. Use the to_csv.py to convert all the bagfiles to .csv files, that are easy to read

2. Use the read_csv_plotting.py file for aligning different data streams and cropping


## to_csv.py
By default, the path in the file assumes that your bagfiles are saved in folder exp_data and your files are named exp_0..100.bag
Change this if you have a different folder structure. It should make folders in your exp_data that contain .csv files for all the ropics you subscribed to.

## read_csv_plotting.py

1. Gets numpy arrays from the .csv files with the get_arrs() function. Make sure the topics dict is up to date for your data and make sure the paths are fixed.'

2. Subsample all data with 0.1 second from reading the timestamps with subsample()

3. Clip all data arrays so they end and start at the same time with get_overlapping_indexes()

4. Filter out points where there is no movement with get_nonzero_indexes()

5. Plotting the resulting data and in a plots folder

6. Crop all long trajectories in smaller ones of 2 seconds (20 samples) and save them in the preprocessed_data2 folder.





