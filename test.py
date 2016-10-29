from NokiaLCD import LCD
import time

if __name__ == "__main__":

    lcd = LCD(dev='/dev/spidev0.1', spd=1e9, reset_pin=40, model='EPS')
    lcd.flush();


    #lcd.display_image('testImg.jpg')
    #lcd.clear_screen((0,0,0))
    #lcd.clear_screen((0,255,0))
    #lcd.clear_screen((0,0,255))
    #lcd.display_off()
    lcd.display_image('testImg.jpg')
    #lcd.display_on()
    time.sleep(1)
    #lcd.clear_screen((0,0,0))
    
    lcd.cleanup()
    

    
