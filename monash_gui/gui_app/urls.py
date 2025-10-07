from django.urls import path
from . import views

urlpatterns = [
    path('health/', views.health_check, name='health_check'),
    path('begin/', views.begin, name='begin'),
    path('autonomous/', views.autonomous, name='autonomous'),
    path('semi_autonomous/', views.semi_autonomous, name='semi_autonomous'),
    path('manual/', views.manual, name='manual'),
    path('choose_object/', views.choose_object, name='choose_object'),
    path('confirm_complete/<str:mode>/', views.confirm_complete, name='confirm_complete'),
    path('mode_selection/', views.mode_selection, name='mode_selection'),
    path('debug_navigation/', views.debug_navigation, name='debug_navigation'),
    path('trial_screen/<int:no_trials>/<int:trial>/', views.trial_screen, name='trial_screen'),
    path('end_screen/', views.end_screen, name='end_screen'),
]