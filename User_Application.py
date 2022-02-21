import time

def print_in_Dot_Matrix_Display(string):
    
    with open("/dev/DotMatrixDisplay", "w") as Dot_Matrix_Display:
        Dot_Matrix_Display.write(string)


if __name__ == "__main__":
    print_in_Dot_Matrix_Display("Test Print")