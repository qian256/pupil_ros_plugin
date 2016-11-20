'''
Example Plugin
- Demonstrating what information is accessible inside a pupil plugin
- Playing with pyglui
- Detailed documentation about g_pool, event, and notification

Author: Long Qian
Email: lqian8@jhu.edu

'''
from plugin import Plugin
from pyglui.cygl.utils import draw_points_norm,RGBA
from pyglui import ui
from glfw import *


class Example_Plugin(Plugin):
    '''
    Example plugin to show what can be done with Pupil plugin

    Attributes:
        self.g_pool:
            Not necessary, the Global_Container shared within world process
        self.alive:
            Whether the plugin is alive. Handled by super class, no need to modify
            it unless necessary.
            if alive is set to False, the plugin will be GCed in world event loop
        self.order:
            Order of execuetion, from 0.0 to 1.0
    '''

    def __init__(self, g_pool):
        '''
        Inherited from base Plugin class
        Information carried by g_pool of world.py:

            General information
            ''g_pool.app'': name of application 'capture' or 'service'
            ''g_pool.user_dir'': user directory, including setting directory
            ''g_pool.version'': version str

            IPC and Synchronization:
            ''g_pool.timebase'': atomic double value of timebase,
                                shared by eye process and word process
            ''g_pool.zmq_ctx'': ZMQ context object
            ''g_pool.ipc_pub'': IPC publisher, for sending notifications
            ''g_pool.ipc_pub_url'': URL str of IPC publisher,
                                for creating gaze_publisher
            ''g_pool.ipc_sub_url'': URL str for IPC subscriber,
                                for creating pupil_sub, and notify_sub
            ''g_pool.ipc_push_url'': URL str for IPC push handler
                                for creating logger
            ''g_pool.get_timestamp'': function to return current timestamp,
                                substracted by shared timebase
            ''g_pool.get_now'': function to return current time
            ''g_pool.eyes_are_alive'': A tuple of two atomic boolean value

            Video source:
            ''g_pool.on_frame_size_change'': A lambda function called when frame size changed
            ''g_pool.capture'': Video source object
                                from [Fake_Source, File_Source, NDSI_Source, UVC_Source]
            ''g_pool.capture_manager'': Video manager object
                                from [Fake_Manager, File_Manager, NDSI_Manager, UVC_Manager]

            GUI elements:
            ''g_pool.main_window'': the main window as GLFW object
            ''g_pool.gui'': containing sidebar, quickbar, graph, and all other GUI components
            ''g_pool.sidebar'': component of g_pool.gui, containing all the menus
            ''g_pool.quickbar'':component of g_pool.gui, containing the thumbs on the left
            ''g_pool.capture_source_menu'':  component of g_pool.gui.sidebar
            ''g_pool.capture_selector_menu'': component of g_pool.gui.sidebar
            ''g_pool.calibration_menu'': component of g_pool.gui.sidebar
            ''g_pool.image_tex'': Image texture from Pupil frame, drawn in main_window,
                                updated at the end of frame, not usable for plugins
            ''g_pool.iconified'': world window iconified or not

            Plugins:
            ''g_pool.plugins'': A list of Plugin objects currently active
            ''g_pool.detection_mapping_mode'': '2d' or '3d', selected from sidebar
            ''g_pool.active_calibration_plugin'': the active calibration plugin,
                                selected from sidebar
            ''g_pool.active_gaze_mapping_plugin'': the active gaze mapping plugin,
                                selected from sidebar, by default, Vector_Gaze_Mapper

        :param g_pool: Global Container of 'world' Process
        '''
        super(Example_Plugin, self).__init__(g_pool)
        self.g_pool = g_pool
        # The update function of this Plugin is executed after other plugins
        self.order = .9

        # GUI elements can be added to the 'world' windows
        self.menu = None
        self.thumb_active = True
        self.growing_menu = None
        self.scrolling_menu = None
        # self.stretching_menu = None
        # self.stretching_menu_thumb_status = True
        self.quickbar_thumb = True
        self.switch_value = True
        self.slider_value = 1.0
        self.selector_items = ['Selector Item 1', 'Selector Item 2', 'Selector Item 3']
        self.selector_idx = 0




    def init_gui(self):
        '''
        Inherited from base Plugin class
        Initialize GUI elements, and add to 'world' window
        Sidebar on the right: g_pool.sidebar
        General Window: g_pool.gui
        Quick bar at left: g_pool.quickbar
        :return: Nothing
        '''
        # Growing Menu
        self.menu = ui.Growing_Menu('Example Plugin')
        self.g_pool.sidebar.append(self.menu)
        self.menu.append(ui.Info_Text('Example GUI elements in pyglui'))
        self.menu.append(ui.Button('Example Button', lambda:self.button_callback()))
        self.menu.append(ui.Button('Print Global Container', lambda:self.print_g_pool_callback()))
        self.menu.append(ui.Switch('switch_value',self, label='Example Switch'))
        self.menu.append(ui.Slider('slider_value',self,step=0.1,min=0.5,max=2.0,label='Example Slider'))
        self.menu.append(ui.Selector('selector_idx',self,selection = range(len(self.selector_items)),
                                     labels=self.selector_items,label='Example Selector'))
        self.menu.append(ui.Button('Remove Plugin', lambda:self.remove_plugin()))

        # Sub-Growing Menu
        self.growing_menu = ui.Growing_Menu('Example Growing Menu')
        self.growing_menu.append(ui.Info_Text('Info Text in Example Growing Menu'))
        self.menu.append(self.growing_menu)
        # Scrolling Menu
        self.scrolling_menu = ui.Scrolling_Menu('Example Scrolling Menu')
        self.scrolling_menu.append(ui.Info_Text('Info Text in Example Scrolling Menu'))
        self.g_pool.gui.append(self.scrolling_menu)
        # Add thumb to quickbar
        self.quickbar_thumb = ui.Thumb('thumb_active', self, label='E', setter=self.thumb_callback, hotkey='e')
        self.quickbar_thumb.on_color[:] = (.3,.2,1.,.9)
        self.g_pool.quickbar.insert(0, self.quickbar_thumb)




    def update(self,frame=None,events={}):
        '''
        called in Player and Capture
        gets called once every frame
        if you plan to update data inplace, note that this will affect all plugins executed after you.
        Use self.order to deal with this appropriately

        :param events: Events contains pupil information and gaze information,
        will be sent to all plugins, with frame data. Plugin is able to modify
        the events list, all added events will be sent to ipc_pub as (topic,
        payload) tuple. Events added by plugins will higher priority (smaller
        value) are visible to low-priority plugins.

        events['dt']:
                Time between current frame and previous frame.
        events['pupil_positions']:
                A list of pupil position data that are generated between current
                frame and previous frame. Note that frame rate of world and eye
                process might be different.
        events['gaze_positions']:
                A list of gaze position data between current frame and previous
                frame. It has same length as 'pupil_positions'

        '''
        # print 'Example Plugin: update'
        # print 'dt', events['dt']
        pass



    def gl_display(self):
        '''
        gets called once every frame when its time to draw onto the gl canvas.
        '''
        pass



    def on_click(self,pos,button,action):
        '''
        Inherited from base Plugin class
        Called when mouse clicked on 'world' window
        including clicking on GUI elements
        :param pos: (x, y) location of mouse when clicking
        :param button: the button on mouse being clicked
        :param action: mouse up or mouse down
        :return: Nothing
        '''
        print 'Example Plugin: on_click called with position:', pos,\
            'left' if button == 0 else ('middle' if button == 2 else 'right'),\
            'up' if action == 0 else 'down'


    def on_window_resize(self,window,w,h):
        '''
        gets called when user resizes window.
        window is the glfw window handle of the resized window.
        '''
        print 'Example Plugin: on_window_resize, resized to %d by %d)' % (w, h)


    def on_notify(self,notification):
        '''
        this gets called when a plugin wants to notify all others.
        notification is a dict in the format {'subject':'notification_category.notification_name',['addional_field':'blah']}
        implement this fn if you want to deal with notifications
        note that notifications are collected from all threads and processes and dispatched in the update loop.
        this callback happens in the main thread.
        '''
        print 'Example Plugin: on_notify'
        print notification


    def get_init_dict(self):
        return {}


    def cleanup(self):
        '''
        gets called when the plugin get terminated.
        This happens either voluntarily or forced.
        if you have an gui or glfw window destroy it here.
        '''
        self.deinit_gui()











    # Customized callback functions
    def button_callback(self):
        print 'Example Plugin: button_callback'


    def thumb_callback(self, _=None):
        '''
        Callback trigger on clicking on GUI thumb, if thumb_active is set to True
        Not triggered if thumb_active is False
        :param _: Placeholder
        :return: Nothing
        '''
        print 'Example Plugin thumb_calledback, thumb_status', 'True' if self.thumb_active else 'False'

    def print_g_pool_callback(self):
        print 'Example Plugin: print_g_pool_callback'
        for attr in dir(self.g_pool):
            print 'obj.%s = %s' % (attr, getattr(self.g_pool, attr))

    def remove_plugin(self):
        self.alive = False

    # def stretching_menu_thumb_callback(self):
    #     print 'Example Plugin stretching_menu_thumb_status'
    #     self.stretching_menu_thumb_status = ~self.stretching_menu_thumb_status


    def deinit_gui(self):
        '''
        Remove GUI elements, used in cleaning up stage
        :return: Nothing
        '''
        if self.menu:
            if self.growing_menu:
                self.menu.remove(self.growing_menu)
                self.growing_menu = None
            self.g_pool.sidebar.remove(self.menu)
            self.menu = None
        if self.scrolling_menu:
            self.g_pool.gui.remove(self.scrolling_menu)
            self.scrolling_menu = None
        if self.quickbar_thumb:
            self.g_pool.quickbar.remove(self.quickbar_thumb)
            self.quickbar_thumb = None

