#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define LCD1_ADDR 0x20
#define LCD2_ADDR 0x21

const int LCD_COLS = 16;
const int LCD_ROWS = 2;

const int n_lcd = 2;
LiquidCrystal_I2C lcd[2] = {
    LiquidCrystal_I2C(LCD1_ADDR, LCD_COLS, LCD_ROWS),
    LiquidCrystal_I2C(LCD2_ADDR, LCD_COLS, LCD_ROWS)};
void lcd_display(int lcd_num, int row, int col, String s);

void setup()
{
    // Khởi động LCD
    for (int i = 0; i < n_lcd; i++)
    {
        lcd[i].init();
        lcd[i].backlight();
    }
}

void loop()
{
    display(0, 0, 7, 1000);
}

void lcd_display(int lcd_num, int row, int col, String s)
{
    lcd[lcd_num].setCursor(col, row);
    lcd[lcd_num].print(s);
}
