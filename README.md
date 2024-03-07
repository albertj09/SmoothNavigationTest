An example Unreal Engine project containing a navigation path smoothing algorithm which tries its' best to produce the best possible looking smooth path based on the original input path (default engine generated path in most cases.)

The algorithm also does dynamic checking and adjusting based on the navmesh, and its' goal is for the smoothed path to be entirely inside navmesh boundaries. This particular part is not 100% guaranteed for now but it's a continous work in progress of finding particular edge cases and working them out.
