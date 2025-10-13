from django.urls import path
from .views import Views

views = Views()

urlpatterns = [
    path('health/', views.health_check, name='health_check'),
    path('', views.begin, name='begin'),
    path('autonomous/<str:msg>/', views.autonomous, name='autonomous'),
    path('semi_autonomous/<str:msg>/', views.semi_autonomous, name='semi_autonomous'),
    path('manual/<str:obj>/<str:msg>/', views.manual, name='manual'),
    path('choose_object/', views.choose_object, name='choose_object'),
    path('confirm_complete/<str:mode>/', views.confirm_complete, name='confirm_complete'),
    path('mode_selection/', views.mode_selection, name='mode_selection'),
    path('debug_navigation/', views.debug_navigation, name='debug_navigation'),
    path('trial_screen/<int:no_trials>/<int:trial>/', views.trial_screen, name='trial_screen'),
    path('end_screen/', views.end_screen, name='end_screen'),
    path('choose_approach_position/', views.choose_approach_position, name='choose_approach_position'),
]