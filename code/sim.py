# -*- coding: utf-8 -*-
"""
Created on Sat Aug 31 10:17:39 2019

@author: elif.ayvali
"""

import scipy.io as sio
import numpy as np
import vtk

class VTK_Tools:    
        
    def addSTL(filename, color,opacity): 
        STL_reader = vtk.vtkSTLReader()
        STL_reader.SetFileName(filename)
        STL_normals = vtk.vtkPolyDataNormals()
        STL_normals.SetFeatureAngle(160)
        STL_normals.SetInputConnection(STL_reader.GetOutputPort())
        mapperSTLscope = vtk.vtkPolyDataMapper()
        mapperSTLscope.SetInputConnection(STL_normals.GetOutputPort())
        STLActor = vtk.vtkActor()
        STLActor.SetMapper(mapperSTLscope)
        STLActor.GetProperty().SetColor(color) # (R,G,B)
        STLActor.GetProperty().SetOpacity(opacity)
        STLActor.GetProperty().SetInterpolationToGouraud()   
        return STLActor
    
    def addAxesMarker(xfm,scalex,scaley,scalez) :
        """ Place axes orientation marker in the scene, transformed by given by vtkTransform
        """
        axesActor = vtk.vtkAxesActor()
        axesActor.SetTotalLength(scalex,scaley,scalez)
        axesActor.SetShaftTypeToCylinder()
        axesActor.AxisLabelsOff()
        axesActor.SetUserTransform(xfm)
        return axesActor
    
    def get_text(text, position_view_port, font_size, color, opacity, bold_flag):
        text_actor = vtk.vtkTextActor()
        text_actor.SetInput(text)
        text_actor.GetTextProperty().SetColor(color)
        text_actor.GetTextProperty().SetBold(bold_flag)
        text_actor.GetTextProperty().SetFontSize(font_size)
        text_actor.GetTextProperty().SetOpacity(opacity)
        text_actor.GetActualPositionCoordinate().SetCoordinateSystemToNormalizedViewport()
        text_actor.SetPosition(position_view_port)
        return text_actor
    
    class CameraData(object):
        __slots__ = ('position', 'focal_point', 'view_up')
    
        def __init__(self, position, focal_point, view_up):
            self.position = position
            self.focal_point = focal_point
            self.view_up = view_up  
            
    def rotmat_to_vtk(mat):
        xfm = vtk.vtkTransform()  
        xfm.PostMultiply()     
        xfm.Identity()
        cnt=0
        Rot = vtk.vtkMatrix4x4()
        Rot.Identity()
        for j in range(3):
            for i in range(3):
                Rot.SetElement(i, j, mat[cnt])   
                cnt+=1                 
        xfm.SetMatrix(Rot)        
        return xfm
    
    
class vtkTimerCallback():

    def __init__(self):
        self.counter = 39000
        self.play_rate=1
        print('Started Rendering')

 
    def execute(self,obj,event):
 
        #get actor handles
        STLActor_mario = self.actors[0]
        STLActor_mario2 = self.actors[1]
        STLActor_mario3 = self.actors[2] 
        STLActor_mario4 = self.actors[3]

        
        #get renderer handles
        ren  = self.renderers[0]
        ren2  = self.renderers[1] 
        ren3  = self.renderers[2]
        ren4  = self.renderers[3] 
        #update rotation         
        rot = self.data[0][self.counter,:]
        rot2 = self.data[1][self.counter,:]
        rot3 = self.data[2][self.counter,:]
        rot4 = self.data[3][self.counter,:]

        
        xfm=VTK_Tools.rotmat_to_vtk(rot)
#        print('rot1',xfm.GetMatrix())
        STLActor_mario.SetUserTransform(xfm) 
        
        xfm2=VTK_Tools.rotmat_to_vtk(rot2)
#        print('rot2',xfm2.GetMatrix())
        STLActor_mario2.SetUserTransform(xfm2)
        
        xfm3=VTK_Tools.rotmat_to_vtk(rot3)
#        print('rot3',xfm3.GetMatrix())
        STLActor_mario3.SetUserTransform(xfm3) 
        
        xfm4=VTK_Tools.rotmat_to_vtk(rot4)
#        print('rot4',xfm4.GetMatrix())
        STLActor_mario4.SetUserTransform(xfm4) 

        ren.ResetCameraClippingRange()
        ren2.ResetCameraClippingRange()
        ren3.ResetCameraClippingRange()
        ren4.ResetCameraClippingRange()


        iren = obj
        iren.GetRenderWindow().Render()
        self.counter += self.play_rate  


def main(data_file_name, stl_name):   
    Data_mat = sio.loadmat(data_file_name) 
    R_exp=Data_mat['R_exp']#nx4
    R_from_nonunit_q_RK4=Data_mat['R_from_nonunit_q_RK4']#nx4
    R_from_q_exp=Data_mat['R_from_q_exp']#nx9
    R_rk4=Data_mat['R_rk4']#nx9
    
    rot_seq=R_rk4
    rot_seq2=R_exp
    rot_seq3=R_from_nonunit_q_RK4
    rot_seq4=R_from_q_exp
    
    xfm_mario = vtk.vtkTransform() 
    xfm_mario.Identity()
    xfm_origin= vtk.vtkTransform() 
    xfm_origin.Identity()
    axes_origin_actor=VTK_Tools.addAxesMarker(xfm_origin,100,100,100)

    #Import Mario STL     
    STLActor_mario=VTK_Tools.addSTL(filename= stl_name,color=[1,1,1],opacity=1)
    STLActor_mario.SetUserTransform(xfm_mario)
    display_flag1 = VTK_Tools.get_text( 'R RK4 integration', position_view_port=[0.40, 0.10], font_size=18, color=[1, 1, 0], opacity=1, bold_flag=1)


    STLActor_mario2=VTK_Tools.addSTL(filename= stl_name,color=[1,1,1],opacity=1)
    STLActor_mario2.SetUserTransform(xfm_mario)
    display_flag2 = VTK_Tools.get_text( 'Exponential matrix update', position_view_port=[0.40, 0.10], font_size=18, color=[1, 1, 0], opacity=1, bold_flag=1)

    STLActor_mario3=VTK_Tools.addSTL(filename= stl_name,color=[1,1,1],opacity=1)
    STLActor_mario3.SetUserTransform(xfm_mario)
    display_flag3 = VTK_Tools.get_text( 'Nonunit q RK4 integration', position_view_port=[0.40, 0.10], font_size=18, color=[1, 1, 0], opacity=1, bold_flag=1)


    STLActor_mario4=VTK_Tools.addSTL(filename= stl_name,color=[1,1,1],opacity=1)
    STLActor_mario4.SetUserTransform(xfm_mario)
    display_flag4 = VTK_Tools.get_text( 'Exponential quaternion update', position_view_port=[0.40, 0.10], font_size=18, color=[1, 1, 0], opacity=1, bold_flag=1)



    #--------------Initialize  World Renderer------------------#
    ren = vtk.vtkRenderer()
    ren.SetBackground(0.14,0.14,0.14)
    ren.SetViewport(0.0, 0.0, 0.5, 0.5) # left bottom quadrant
    ren.GradientBackgroundOn();
    ren.AddActor(STLActor_mario)  
    ren.AddActor(axes_origin_actor)
    ren.AddActor(display_flag1)
    
    ren2 = vtk.vtkRenderer()
    ren2.SetBackground(0.14,0.14,0.14)
    ren2.SetViewport(0.5, 0.0, 1.0, 0.5) # right bottom quadrant
    ren2.GradientBackgroundOn();
    ren2.AddActor(STLActor_mario2)  
    ren2.AddActor(axes_origin_actor)
    ren2.AddActor(display_flag2)
    
    ren3 = vtk.vtkRenderer()
    ren3.SetBackground(0.14,0.14,0.14)
    ren3.SetViewport(0.0, 0.5, 0.5, 1.0) # left top quadrant
    ren3.GradientBackgroundOn();
    ren3.AddActor(STLActor_mario3)  
    ren3.AddActor(axes_origin_actor)
    ren3.AddActor(display_flag3)
    
    ren4 = vtk.vtkRenderer()
    ren4.SetBackground(0.14,0.14,0.14)
    ren4.SetViewport(0.5, 0.5, 1.0, 1.0) # right top quadrant
    ren4.GradientBackgroundOn();
    ren4.AddActor(STLActor_mario4)  
    ren4.AddActor(axes_origin_actor)
    ren4.AddActor(display_flag4)


    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(ren)
    renWin.AddRenderer(ren2)
    renWin.AddRenderer(ren3)
    renWin.AddRenderer(ren4)


    # Create a renderwindowinteractor#
    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(renWin)
    iren.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
    #Enable user interface interactor
    iren.Initialize()
    
    #variables to pass into the callback class cb
    actorCollection = []    
    actorCollection.append(STLActor_mario)    
    actorCollection.append(STLActor_mario2)    
    actorCollection.append(STLActor_mario3)    
    actorCollection.append(STLActor_mario4)    


    rendererCollection=[]
    rendererCollection.append(ren)      
    rendererCollection.append(ren2)   
    rendererCollection.append(ren3)      
    rendererCollection.append(ren4)      


    dataCollection=[]
    dataCollection.append(rot_seq)
    dataCollection.append(rot_seq2)
    dataCollection.append(rot_seq3)
    dataCollection.append(rot_seq4)


    
    # Sign up to receive TimerEvent
    cb = vtkTimerCallback()
    cb.actors=actorCollection;
    cb.data=dataCollection;
    #cb.cameras=cameraCollection;
    cb.renderers=rendererCollection;
    iren.AddObserver('TimerEvent', cb.execute)
    iren.CreateRepeatingTimer(150);
    #start the interaction and timer
    iren.Start()    
    iren.DestroyTimer()
    
  
if __name__ == '__main__':
   main(data_file_name='test_data3.mat', stl_name = 'mario.stl')
