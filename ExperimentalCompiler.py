import numpy as np
import cv2

class Compiler:
    ## COMPILATION SUBSTRINGS
    final_servo_code: str
    servo_init_script: str
    memory_allocation_script: str
    array_init_script: str
    pls_init_script: str
    purge_script: str
    print_loop_script: str

    ## INPUT PARAMETERS
    drop_diameter: int # micron
    x_home: int # encoder count
    y_home: int # encoder count
    x_ramp_array_size: int # number of radius
    prefire_offset: int # encoder count
    total_offset: int # encoder count
    purge_row_offset: int # purge row located above the first row, by encoder count
    acc: int # count/s^2
    dec: int # count/s^2
    stp: int # count/s^2
    vel: int # count/s
    image_profile: np.ndarray
    encoder_conversion_factor: float # micron per count

    ## INTERNAL CALCULATIONS
    masked_image: np.ndarray
    true_x_home: int # count
    right_most_x: int # count
    row_canvas_array_size: int # number of drop diameter
    num_row: int # number of rows
    y_step: int # count
    row_array_size: int

    macro_template = (
        "PROGRAM\n"
        "\n"
        "{servo_init_sub:s}"
        "\n"
        "{memory_alloc:s}"
        "\n"
        "{array_state_init:s}"
        "\n"
        "{pls_init:s}"
        "\n"
        "{purge:s}"
        "\n"
        "{print_loop:s}"
        "\n"
        "END\n"
        "ENDP\n"
    )


    def __init__(self,
    drop_diameter: int, # micron
    x_home: int,
    y_home: int,
    x_ramp_array_size: int,
    prefire_offset: int,
    purge_row_offset: int,
    acc: int,
    dec: int,
    stp: int,
    vel: int,
    image_profile: np.ndarray,
    encoder_conversion_factor: float) -> None:

        # Initialize Internal State
        self.drop_diameter = drop_diameter
        self.x_home = x_home
        self.y_home = y_home
        self.x_ramp_array_size = x_ramp_array_size
        self.prefire_offset = prefire_offset
        self.purge_row_offset = purge_row_offset
        self.acc = acc
        self.dec = dec
        self.stp = stp
        self.vel = vel
        self.image_profile = image_profile
        self.encoder_conversion_factor = encoder_conversion_factor

        # Internal Calculations
        self.masked_image_generation()
        self.y_step_calculation()
        self.total_offset_calculation()
        self.true_x_home_with_offset_calculation()
        self.array_size_allocation()
        self.right_most_destination_calculation()

        # Servo Code Generation By Segments
        self.servo_init_script_generation()
        self.memory_allocation_script_generation()
        self.array_state_init_script_generation()
        self.pls_init_script_generation()
        self.purge_movement_script_generation()
        self.array_loop_script_generator()

        self.final_servo_code = self.macro_template.format(
            servo_init_sub = self.servo_init_script,
            memory_alloc = self.memory_allocation_script,
            array_state_init = self.array_init_script,
            pls_init = self.pls_init_script,
            purge = self.purge_script,
            print_loop = self.print_loop_script
        )

    def masked_image_generation(self) -> None:
        mask_dim  = self.image_profile.shape
        checkered_array = np.zeros(mask_dim, dtype = np.uint8)
        checkered_array[::2,::2] = 255
        checkered_array[1::2,1::2] = 255
        self.masked_image = cv2.bitwise_and(self.image_profile,self.image_profile,mask = checkered_array)
    
    def y_step_calculation(self) -> None:
        self.y_step = int(self.drop_diameter*np.sin(np.pi/3)/self.encoder_conversion_factor)

    def total_offset_calculation(self) -> None:
        self.total_offset = self.prefire_offset + int(self.drop_diameter / 2 /self.encoder_conversion_factor)

    def true_x_home_with_offset_calculation(self) -> None:
        self.true_x_home = self.x_home - self.total_offset

    def array_size_allocation(self) -> None:
        self.row_canvas_array_size = self.image_profile.shape[1]
        self.num_row = self.image_profile.shape[0]
        self.row_array_size = self.x_ramp_array_size*2 + self.row_canvas_array_size

    def right_most_destination_calculation(self) -> None:
        self.right_most_x = int((self.drop_diameter/self.encoder_conversion_factor)*self.row_array_size + self.total_offset)

    def servo_init_script_generation(self):
        self.servo_init_script = (
            "AXIS0 PPU 1\n"
            "AXIS1 PPU 1\n\n"

            f"ACC {self.acc:d} DEC {self.dec:d} STP {self.stp:d} VEL {self.vel:d}\n\n"

            f"X{self.true_x_home:d} Y{self.y_home + self.purge_row_offset:d}\n"           
        )

    def memory_allocation_script_generation(self):
        working_str = "DIM LV(1)\nDIM LA(2)\n"
        row_array_init_string = (
            f"DIM LA0({self.row_array_size:d})\n"
            f"DIM LA1({self.row_array_size:d})\n"
        )

        self.memory_allocation_script = working_str + row_array_init_string


    def array_state_init_script_generation(self) -> None:
        command_init_even = (
            f"FOR LV0 = 0 TO {self.x_ramp_array_size // 2 - 1:d} STEP 2\n"
            "    LA0(LV0) = 1\n"
            "    NEXT\n"
            f"FOR LV0 = 1 TO {self.x_ramp_array_size // 2 - 1:d} STEP 2\n"
            "    LA0(LV0) = 0\n"
            "    NEXT\n"
            f"FOR LV0 = {self.x_ramp_array_size // 2:d} TO {self.x_ramp_array_size - 1:d} STEP 1\n"
            "    LA0(LV0) = 0\n"
            "    NEXT\n"
            f"FOR LV0 = {self.x_ramp_array_size:d} TO {self.x_ramp_array_size + self.row_canvas_array_size - 1:d} STEP 2\n"
            "    LA0(LV0) = 1\n"
            "    NEXT\n"
            f"FOR LV0 = {self.x_ramp_array_size + 1:d} TO {self.x_ramp_array_size + self.row_canvas_array_size - 1:d} STEP 2\n"
            "    LA0(LV0) = 0\n"
            "    NEXT\n"
            f"FOR LV0 = {self.x_ramp_array_size + self.row_canvas_array_size:d} TO {self.x_ramp_array_size + self.row_canvas_array_size + self.x_ramp_array_size // 2 - 1:d} STEP 1\n"
            "    LA0(LV0) = 0\n"
            "    NEXT\n"
            f"FOR LV0 = {self.x_ramp_array_size + self.row_canvas_array_size + self.x_ramp_array_size // 2:d} TO {self.x_ramp_array_size*2 + self.row_canvas_array_size - 1:d} STEP 2\n"
            "    LA0(LV0) = 1\n"
            "    NEXT\n"
            f"FOR LV0 = {self.x_ramp_array_size + self.row_canvas_array_size + self.x_ramp_array_size // 2 + 1:d} TO {self.x_ramp_array_size*2 + self.row_canvas_array_size - 1:d} STEP 2\n"
            "    LA0(LV0) = 0\n"
            "    NEXT\n\n"
        )
        
        command_init_odd = (
            f"FOR LV0 = 0 TO {self.x_ramp_array_size // 2 - 1:d} STEP 2\n"
            "    LA1(LV0) = 1\n"
            "    NEXT\n"
            f"FOR LV0 = 1 TO {self.x_ramp_array_size // 2 - 1:d} STEP 2\n"
            "    LA1(LV0) = 0\n"
            "    NEXT\n"
            f"FOR LV0 = {self.x_ramp_array_size // 2:d} TO {self.x_ramp_array_size - 1:d} STEP 1\n"
            "    LA1(LV0) = 0\n"
            "    NEXT\n"
            f"FOR LV0 = {self.x_ramp_array_size:d} TO {self.x_ramp_array_size + self.row_canvas_array_size - 1:d} STEP 2\n"
            "    LA1(LV0) = 0\n"
            "    NEXT\n"
            f"FOR LV0 = {self.x_ramp_array_size + 1:d} TO {self.x_ramp_array_size + self.row_canvas_array_size - 1:d} STEP 2\n"
            "    LA1(LV0) = 1\n"
            "    NEXT\n"
            f"FOR LV0 = {self.x_ramp_array_size + self.row_canvas_array_size:d} TO {self.x_ramp_array_size + self.row_canvas_array_size + self.x_ramp_array_size // 2 - 1:d} STEP 1\n"
            "    LA1(LV0) = 0\n"
            "    NEXT\n"
            f"FOR LV0 = {self.x_ramp_array_size + self.row_canvas_array_size + self.x_ramp_array_size // 2:d} TO {self.x_ramp_array_size*2 + self.row_canvas_array_size - 1:d} STEP 2\n"
            "    LA1(LV0) = 1\n"
            "    NEXT\n"
            f"FOR LV0 = {self.x_ramp_array_size + self.row_canvas_array_size + self.x_ramp_array_size // 2 + 1:d} TO {self.x_ramp_array_size*2 + self.row_canvas_array_size - 1:d} STEP 2\n"
            "    LA1(LV0) = 0\n"
            "    NEXT\n"
        )
        
        self.array_init_script = command_init_even + command_init_odd
        

    def pls_init_script_generation(self) -> None:
        ratio = self.encoder_conversion_factor / (self.drop_diameter)
        self.pls_init_script = (
            "PLS0 SRC P12290\n"
            "PLS0 FLZ 0\n"
            f"PLS0 RATIO {ratio:.6f}\n"
            "PLS0 DST P4216\n"
            "PLS0 MASK 1\n"
            "PLS0 ON\n"
        )

    def purge_movement_script_generation(self) -> None:
        self.purge_script = (
            "RES X0\n"
            "PLS0 BASE LA0\n"
            f"X{self.right_most_x}\n"
            "INH -516\n"
            "X0\n"
            "INH -516\n"
            f"Y{self.y_home}\n"
        )

    def array_loop_script_generator(self) -> None:

        prev_odd = np.ones(self.row_canvas_array_size)
        prev_odd[::2] = 0

        prev_even = np.ones(self.row_canvas_array_size)
        prev_even[1::2] = 0

        command = ""

        first_row = self.masked_image[0,:]//255
        even_init_delta = first_row - prev_even
        prev_even = first_row
        for entry_index, entry in enumerate(even_init_delta):
            if entry == 0:
                continue
            else:
                true_index = entry_index + self.x_ramp_array_size
                command += f"LA0({true_index:d}) = {first_row[entry_index]:d}\n"
        
        prev_even = self.masked_image[0,:]
        command += (
            "INH -516\n"
            "\nRES X0\n"
            "PLS0 BASE LA0\n"           
        )

        for row_index, row in enumerate(self.masked_image[1:,:]//255,1):
            if row_index % 2 == 0:
                image_delta = row - prev_even
                prev_even = row
                command += f"X{-self.total_offset:d}\n"
                for entry_index, entry in enumerate(image_delta):
                    if entry == 0:
                        continue
                    else:
                        true_index = entry_index + self.x_ramp_array_size
                        command += f"LA0({true_index:d}) = {row[entry_index]:d}\n"
                command += (
                    "INH -516\n"
                    f"\nRES X0\n"
                    f"Y/{self.y_step:d}\n"
                    "INH -516\n"
                    "PLS0 BASE LA0\n"            
                )

            else:
                image_delta = row - prev_odd
                prev_odd = row
                command += f"X{self.right_most_x:d}\n"
                for entry_index, entry in enumerate(image_delta):
                    if entry == 0:
                        continue
                    else:
                        true_index = entry_index + self.x_ramp_array_size
                        command += f"LA1({true_index:d}) = {row[entry_index]:d}\n"
                command += (
                    "INH -516\n"
                    f"\nRES X{self.right_most_x - self.total_offset:d}\n"
                    f"Y/{self.y_step:d}\n"
                    "INH -516\n"
                    "PLS0 BASE LA1\n"                    
                )

        if (self.num_row % 2) == 0:
            command += f"X{-self.total_offset:d}\nINH -516\n"
        else:
            command += f"X{self.right_most_x:d}\nINH -516\n"
        command += (
            "PLS0 OFF\n"
            "X0 Y0\n"
        )
        self.print_loop_script = command
    
    def export_to_txt(self, file_name: str):
        with open(file_name, "w") as writer:
            writer.write(self.final_servo_code)