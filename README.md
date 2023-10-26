# FoveaCam Controller Firmware

### References

+ Mirrorcle MEMS Data Sheet: [A5L3.3(C2)-6400AU](https://www.mirrorcletech.com/pdf/DS/MirrorcleTech_Datasheet_A5L3.3(C2)-6400AU.pdf)

+ MEMS Driver IC Reference: [AD5264R](https://www.analog.com/media/en/technical-documentation/data-sheets/AD5624R_5644R_5664R.pdf)

### Useful Commands

+ **Build project**

    ```sh
    pio run
    ```

+ **Generate compile_commands.json** (for clangd)

    ```sh
    pio run --target compiledb
    ```

+ **Compile and Upload**

    ```sh
    pio run --target upload
    ```

+ **Clean build files**

    ```sh
    pio run --target clean
    ```
