#ifndef CASRECONSTRUCTIONFACTORY_H
#define CASRECONSTRUCTIONFACTORY_H

#include "CASReconstructionGeneral.h"
#include "CASReconstructionCA3DoF.h"
#include "CASReconstructionCA2DoF.h"
#include "CASReconstructionCV3DoF.h"
#include "CASReconstructionCV2DoF.h"
#include "CASAbstractReconstruction.h"

namespace cas
{
    class CASReconstructionFactory
    {
        public:
            static CASAbstractReconstruction* Create(const char* type, int params_num = -1, char* msg = NULL);
    };

    CASAbstractReconstruction* CASReconstructionFactory::Create(const char* type, int params_num, char* msg)
    {
        if(!strcmp(type, "general"))
        {            
            if ((params_num != -1) && (params_num < 4))
            {
                if (msg!=NULL)
                    strcpy(msg, "Velocities matrix has to contain at least 4 columns for the general model: T VX VY VZ!");
                return NULL;
            }         
            
            CASAbstractReconstruction* reconstruction = new CASReconstructionGeneral();
            return reconstruction;
        }    
        else if(!strcmp(type, "CA3DoF"))
        {            
            if ((params_num != -1) && (params_num < 6))
            {
                if (msg!=NULL)
                    strcpy(msg, "Motion vector has to contain at least 6 columns for the CA3DoF model: VX VY VZ AX AY AZ!\nFor angle correction the rotation center has to be known: VX VY VZ AX AY AZ CX CY");
                return NULL;
            }
            if (params_num < 8)
            {
                if (msg!=NULL)
                    strcpy(msg, "WARNING: Angle correction is not applied.\nFor angle correction the rotation center has to be known: VX VY VZ AX AY AZ CX CY.");                
            }

            CASAbstractReconstruction* reconstruction = new CASReconstructionCA3DoF();
            return reconstruction;

        }    
        else if(!strcmp(type, "CA2DoF"))
        {            
            if ((params_num != -1) && (params_num < 4) && (msg!=NULL))
            {
                if (msg!=NULL)
                    strcpy(msg, "Motion vector has to contain at least 4 columns for the CA2DoF model: VX VY AX AY!\nFor angle correction the rotation center has to be known: VX VY AX AY CX CY");                
                return NULL;
            }       
            
            if (params_num < 6)
            {
                if (msg!=NULL)
                    strcpy(msg, "WARNING: Angle correction is not applied.\nFor angle correction the rotation center has to be known: VX VY AX AY CX CY.");                
            }

            CASAbstractReconstruction* reconstruction = new CASReconstructionCA2DoF();
            return reconstruction;

        }
        else if(!strcmp(type, "CV3DoF"))
        {                    
            if ((params_num != -1) && (params_num < 3) && (msg!=NULL))
            {
                if (msg!=NULL)
                    strcpy(msg, "Motion vector has to contain at least 3 columns for the CA2DoF model: VX VY VZ!");
                return NULL;
            }         

            CASAbstractReconstruction* reconstruction = new CASReconstructionCV3DoF();
            return reconstruction;
        }
        else if(!strcmp(type, "CV2DoF"))
        {
            if ((params_num != -1) && (params_num < 2) && (msg!=NULL))
            {
                if (msg!=NULL)
                    strcpy(msg, "Motion vector has to contain at least 2 columns for the CA2DoF model: VX VY!");
                return NULL;
            }         

            CASAbstractReconstruction* reconstruction = new CASReconstructionCV2DoF();
            return reconstruction;
        }
        else
        {
            if (msg!=NULL)
                strcpy(msg, "Model is not known! Available models: general, CA3DoF, CA2DoF, CV3DoF, CV2DoF.");
            return NULL;
        }    
    }
}

#endif