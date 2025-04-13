def DrawBoardFORREMOVE(rows, cols, pxarray, pallet, C2C, clear, r):
    buff_mod = int(clear + r)
    for x in range(0,rows):
        for y in range(0,cols):
            in_obj = InObjectSpace(x,y)
            if (in_obj):
                pxarray[x,y] = pygame.Color(pallet["black"])
            else:
                if(((InObjectSpace(x+(buff_mod*0.5),y+buff_mod)) or\
                   (InObjectSpace(x-(buff_mod*0.5),y+buff_mod)) or\
                   (InObjectSpace(x+(buff_mod*0.5),y-buff_mod)) or\
                   (InObjectSpace(x-(buff_mod*0.5),y-buff_mod)) or\
                   (InObjectSpace(x,y+buff_mod)) or\
                   (InObjectSpace(x,y-buff_mod)) or\
                   (InObjectSpace(x+buff_mod,y+buff_mod)) or\
                   (InObjectSpace(x-buff_mod,y+buff_mod)) or \
                   (InObjectSpace(x+buff_mod,y-buff_mod) and (y < 149)) or \
                   (InObjectSpace(x-buff_mod,y-buff_mod) and (y < 149)) or \
                   (InObjectSpace(x+buff_mod,y-buff_mod) and ((209-buff_mod)<=x<=(219+buff_mod))) or \
                   (InObjectSpace(x-buff_mod,y-buff_mod) and ((209-buff_mod)<=x<=(219+buff_mod)))) and \
                   ((buff_mod<x<(538-buff_mod) and (buff_mod<y<(298-buff_mod))))):
                    pxarray[x,y] = pygame.Color(pallet["green"])
                elif(0<y<=buff_mod or (298-buff_mod)<=y<299):
                     pxarray[x,y] = pygame.Color(pallet["green"])
                else:
                    pxarray[x,y] = pygame.Color(pallet["white"])
