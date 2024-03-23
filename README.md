An example Unreal Engine project containing a navigation path smoothing algorithm which tries its' best to produce the best possible looking smooth path based on the original input path (default engine generated path in most cases).

The algorithm also does dynamic checking and adjusting based on the navmesh, and its' goal is for the smoothed path to be entirely inside navmesh boundaries. This particular part is not 100% guaranteed for now but it's a continous work in progress of finding particular edge cases and working them out.

Some screenshots:

![image](https://github.com/albertj09/SmoothNavigationTest/assets/46754230/d8a785c9-e648-4a3e-bf9a-95095fd30401)
![image](https://github.com/albertj09/SmoothNavigationTest/assets/46754230/2617429d-e410-4276-9f0c-de361d332012)
![image](https://github.com/albertj09/SmoothNavigationTest/assets/46754230/ad8e96db-8a0f-4b5d-8926-4834a3c41fe9)

Demo video #1:
https://youtu.be/xf3yGopcDPA

Demo video #2:
https://youtu.be/v0GwR_COjaM

Demo video of the debugging tools:
https://youtu.be/n5dcTpCvNn0
