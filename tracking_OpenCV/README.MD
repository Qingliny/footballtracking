1. We first read the soccer game frame by frame and then convert all the frames into HSV format based images. By using HSV image can we detect any specified color background ranging from specific codes. In this project, we first need to detect the green field using HSV technique, and the other parts will in general become black. After that, it is much easier to detect players by jersey colors.
 
2. We define a mask of a given green color range to detect the field. The players and audiences are shown in black, the green field is preserved  within our given green color range.


3. Find all contours on every frame. We consider contours whose height is greater than width as detected player. For recognition of jersey color, we can do the same. Namely, we do masking operation on the detected players to distinguish different jersey colors. For simplicity, we just put a rectangle with blue or red colors representing blue or red teams to each person.

4. For detecting soccer ball, we detect white color using template matching with a suitable mask. The performance for this soccer detection, remains to be improved in the future as listed in the future work of the project report.
