In order to place trees (or crops) in ways that farmers actually place them, we need to align the crop
 rows with the parcel in which they are placed. In order to do this, randomly select (for each parcel)
 a vertex of the parcel for the crop placement starting point. Now randomly pick an adjacent parcel edge
as the "row" edge. Trees will now be placed along rows parallel to this "row" edge, in columns parallel
 to the other edge.
In order to visually test this layout, we need smaller tree spacing relative to the test layout - which is typically 100m (or 60, for your tests). So the default (or test) tree spacing should be changed to 1m, and the 
headland_width to  2m. Change the tree row_spacing to 1.5m. Regenerate the debug views and run the tests.
